#include "viewer.h"
#include "build/ui_viewer.h"

#define IOU_T 0.5

CloudViewer::CloudViewer(QWidget *parent):
  QMainWindow(parent),
  ui(new Ui::CloudViewer) {
  ui->setupUi(this);
  this->setWindowIcon(QIcon(":/images/lcas_logo.png"));
  this->setWindowTitle("L-CAS Cloud Annotation Tool 2");
  load_file_path = std::getenv("HOME");
  
  trunk_show = false;
  slice_show = false;
  bins_show = false;

  adaptive_clustering = false;
  show_intensity = false;
  auto_next = false;
  
  remove_top_bottom = true;
  down_sampling = false;
  remove_outliers = false;
  remove_planes = false;
  
  // Set up the QVTK window.
  viewer.reset(new pcl::visualization::PCLVisualizer("PCL Viewer", false));
  ui->qvtkWidget->SetRenderWindow(viewer->getRenderWindow());
  viewer->setupInteractor(ui->qvtkWidget->GetInteractor(), ui->qvtkWidget->GetRenderWindow());
  ui->qvtkWidget->update();
  
  // Connect UI and their functions.
  connect(ui->pushButton_load, SIGNAL(clicked()), this, SLOT(loadButtonClicked()));
  connect(ui->pushButton_label, SIGNAL(clicked()), this, SLOT(labelButtonClicked()));
  connect(ui->pushButton_extract, SIGNAL(clicked()), this, SLOT(extractButtonClicked()));
  connect(ui->pushButton_background, SIGNAL(clicked()), this, SLOT(backgroundButtonClicked()));
  connect(ui->pushButton_train, SIGNAL(clicked()), this, SLOT(trainButtonClicked()));
  connect(ui->pushButton_predict, SIGNAL(clicked()), this, SLOT(predictButtonClicked()));
  connect(ui->pushButton_reload, SIGNAL(clicked()), this, SLOT(reloadButtonClicked()));

  connect(ui->checkBox_adaptive, SIGNAL(stateChanged(int)), this, SLOT(adaptiveBoxChecked()));
  connect(ui->checkBox_intensity, SIGNAL(stateChanged(int)), this, SLOT(intensityBoxChecked()));
  connect(ui->checkBox_next, SIGNAL(stateChanged(int)), this, SLOT(nextBoxChecked()));
  connect(ui->checkBox_trunk, SIGNAL(stateChanged(int)), this, SLOT(trunkBoxChecked()));
  connect(ui->checkBox_slice, SIGNAL(stateChanged(int)), this, SLOT(sliceBoxChecked()));
  connect(ui->checkBox_bins, SIGNAL(stateChanged(int)), this, SLOT(binsBoxChecked()));
  
  connect(ui->listWidget_files, SIGNAL(itemSelectionChanged()), this, SLOT(fileItemChanged()));
}

CloudViewer::~CloudViewer() {
  delete ui;
}

double s2d(std::string s) {
  std::stringstream ss;
  double d;
  ss << s;
  ss >> d;
  return d;
}

float iou3d(float axmin, float aymin, float azmin,
	    float axmax, float aymax, float azmax,
	    float bxmin, float bymin, float bzmin,
	    float bxmax, float bymax, float bzmax) {
  float xa = std::max(axmin, bxmin);
  float ya = std::max(aymin, bymin);
  float za = std::max(azmin, bzmin);
  
  float xb = std::min(axmax, bxmax);
  float yb = std::min(aymax, bymax);
  float zb = std::min(azmax, bzmax);
  
  float interVolume = std::max(0.0, double(xb-xa)) * std::max(0.0, double(yb-ya)) * std::max(0.0, double(zb-za));
  
  float boxAVolume = (axmax - axmin) * (aymax - aymin) * (azmax - azmin);
  float boxBVolume = (bxmax - bxmin) * (bymax - bymin) * (bzmax - bzmin);
  
  return (interVolume / (boxAVolume + boxBVolume - interVolume));
}

void writeData(std::fstream &file, std::vector<Feature>::iterator &it, int class_lable) {
  file << class_lable <<
    " 1:"  << it->number_points <<
    " 2:"  << it->min_distance <<
    " 3:"  << it->covariance_3d(0,0) <<
    " 4:"  << it->covariance_3d(0,1) <<
    " 5:"  << it->covariance_3d(0,2) <<
    " 6:"  << it->covariance_3d(1,1) <<
    " 7:"  << it->covariance_3d(1,2) <<
    " 8:"  << it->covariance_3d(2,2) <<
    " 9:"  << it->moment_3d(0,0) <<
    " 10:" << it->moment_3d(0,1) <<
    " 11:" << it->moment_3d(0,2) <<
    " 12:" << it->moment_3d(1,1) <<
    " 13:" << it->moment_3d(1,2) <<
    " 14:" << it->moment_3d(2,2);
  for(int i = 0; i < 9; i++) {
    file << " " << (i+15) << ":" << it->partial_covariance_2d[i];
  }
  for(int i = 0; i < 98; i++) {
    file << " " << (i+24) << ":" << it->histogram_main_2d[i];
  }
  for(int i = 0; i < 45; i++) {
    file << " " << (i+122) << ":" << it->histogram_second_2d[i];
  }
  for(int i = 0; i < 30; i++) {
    file << " " << (i+167) << ":" << it->slice[i];
  }
  for(int i = 0; i < 27; i++) {
    file << " " << (i+197) << ":" << it->intensity[i];
  }
  file << "\n";
}

void CloudViewer::loadButtonClicked() {
  // Load pcd files.
  QStringList files = QFileDialog::getOpenFileNames(this, tr("Select one or more files to open"), load_file_path, tr("Point cloud data(*.pcd)"));
  if(files.isEmpty()) {
    return;
  }
  load_file_path = QFileInfo(files[0]).absolutePath();
  
  // Show file list.
  for(QStringList::Iterator it = files.begin(); it != files.end(); ++it) {
    QList<QListWidgetItem*> items = ui->listWidget_files->findItems(*it, Qt::MatchExactly);
    if(items.size() == 0) {
      ui->listWidget_files->addItem(*it);
    }
  }
  if(ui->listWidget_files->currentRow() == -1) {
    ui->listWidget_files->setCurrentRow(0);
  }
}

void CloudViewer::labelButtonClicked() {
  if(ui->listWidget_files->currentRow() == -1) {
    return;
  }
  
  if(!QDir(current_label_path).exists()) {
    QDir().mkpath(current_label_path);
  }
  
  std::string string_to, line_to, line_in;
  for(std::vector<Feature>::iterator it = features.begin(); it != features.end(); ++it) {
    if(boost::to_string(it->id) == ui->lineEdit_object_id->text().toStdString()) {
      bool new_label = true, change_label = false;
      line_to = boost::to_string(it->centroid[0])+" "+boost::to_string(it->centroid[1])+" "+boost::to_string(it->centroid[2])+" "+
	boost::to_string(it->min[0])+" "+boost::to_string(it->min[1])+" "+boost::to_string(it->min[2])+" "+
	boost::to_string(it->max[0])+" "+boost::to_string(it->max[1])+" "+boost::to_string(it->max[2]);
      label_file.open((current_label_path+"/"+QFileInfo(ui->listWidget_files->currentItem()->text()).completeBaseName()+".txt").toStdString().c_str(), std::fstream::in);
      while(std::getline(label_file, line_in)) {
	std::vector<std::string> params;
	boost::split(params, line_in, boost::is_any_of(" "));
	if(it->centroid[0] > s2d(params[4]) && it->centroid[0] < s2d(params[7]) &&
	   it->centroid[1] > s2d(params[5]) && it->centroid[1] < s2d(params[8]) &&
	   it->centroid[2] > s2d(params[6]) && it->centroid[2] < s2d(params[9])) {
	  //if(line_in.substr(line_in.find(" ")+1, line_in.length()-line_in.find(" ")-3).compare(line_to) == 0) {
	  new_label = false;
      	  if(ui->comboBox_class->currentText().toStdString().compare("dontcare") == 0) {
      	    viewer->removeText3D("labeled_text_"+boost::to_string(it->centroid[0]));
      	    viewer->removeShape("labeled_box_"+boost::to_string(it->centroid[0]));
      	    ui->label_show->setText("<font color=\"blue\">Label removed.</font>");
	    continue;
      	  } else {
      	    if(ui->comboBox_class->currentText().toStdString().compare(line_in.substr(0, line_in.find(" "))) == 0 &&
	       boost::to_string(ui->comboBox_visibility->currentIndex()).compare(line_in.substr(line_in.length()-1)) == 0) {
      	      ui->label_show->setText("<font color=\"blue\">Do nothing.</font>");
      	    } else {
	      change_label = true;
	      viewer->removeText3D("labeled_text_"+boost::to_string(it->centroid[0]));
	      viewer->removeShape("labeled_box_"+boost::to_string(it->centroid[0]));
	      ui->label_show->setText("<font color=\"blue\">Label changed.</font>");
	      continue;
      	    }
      	  }
      	}
	string_to += line_in+"\n";
      }
      if(new_label || change_label) {
	if(ui->comboBox_class->currentText().toStdString().compare("dontcare") == 0) {
	  ui->label_show->setText("<font color=\"blue\">Do nothing.</font>");	
	} else {
	  string_to += ui->comboBox_class->currentText().toStdString()+" "+line_to+" "+
	    boost::to_string(ui->comboBox_visibility->currentIndex())+"\n";
	  double r, g, b;
	  if(ui->comboBox_class->currentText().toStdString().compare("pedestrian") == 0) {r=1; g=0; b=0;}
	  if(ui->comboBox_class->currentText().toStdString().compare("group") == 0)      {r=0; g=1; b=0;}
	  if(ui->comboBox_class->currentText().toStdString().compare("wheelchair") == 0) {r=0; g=0; b=1;}
	  if(ui->comboBox_class->currentText().toStdString().compare("cyclist") == 0)    {r=1; g=1; b=0;}
	  if(ui->comboBox_class->currentText().toStdString().compare("car") == 0)        {r=0; g=1; b=1;}
	  pcl::PointXYZ pos(it->centroid[0], it->centroid[1], it->max[2]);
	  viewer->addText3D(ui->comboBox_class->currentText().toStdString(), pos, 0.2, r, g, b, "labeled_text_"+boost::to_string(it->centroid[0]));
	  viewer->addCube(it->min[0], it->max[0], it->min[1], it->max[1], it->min[2], it->max[2], r, g, b, "labeled_box_"+boost::to_string(it->centroid[0]));
	  viewer->setRepresentationToWireframeForAllActors();
	  viewer->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_LINE_WIDTH, 4, "labeled_box_"+boost::to_string(it->centroid[0]));
	  if(change_label == false) {
	    ui->label_show->setText("<font color=\"blue\">Label added.</font>");
	  }
	}
      }
      label_file.close();
      label_file.open((current_label_path+"/"+QFileInfo(ui->listWidget_files->currentItem()->text()).completeBaseName()+".txt").toStdString().c_str(), std::fstream::out | std::fstream::trunc);
      label_file << string_to;
      label_file.close();
      ui->qvtkWidget->update();
      if(auto_next) {
	ui->listWidget_files->setCurrentRow(ui->listWidget_files->currentRow()+1);
      }
      return;
    }
  }
  ui->label_show->setText("<font color=\"red\">Unknown ID.</font>");
}

void CloudViewer::extractButtonClicked() { // @todo use svm.h
  if(!QDir("svm").exists()) {
    QDir().mkdir("svm");
  }
  
  if(ui->comboBox_policy->currentIndex() == 0) { // single object
    for(std::vector<Feature>::iterator it = features.begin(); it != features.end(); ++it) {
      if(boost::to_string(it->id) == ui->lineEdit_object_id->text().toStdString()) {
	feature_file.open(("svm/"+ui->comboBox_class->currentText()).toStdString().c_str(), std::fstream::in | std::fstream::out | std::fstream::app);
	writeData(feature_file, it, 1);
	feature_file.close();
	ui->label_show->setText("<font color=\"blue\">Feature extracted.</font>");
	if(auto_next) {
	  ui->listWidget_files->setCurrentRow(ui->listWidget_files->currentRow()+1);
	}
	return;
      }
    }
    ui->label_show->setText("<font color=\"red\">Unknown ID.</font>");
  }
  
  if(ui->comboBox_policy->currentIndex() == 1) { // single file
    if(file_labeled) {
      feature_file.open(("svm/"+ui->comboBox_class->currentText()).toStdString().c_str(), std::fstream::in | std::fstream::out | std::fstream::app);
      int n = 0;
      for(int i = 0; i < labels.size(); i++) {
      	if(labels[i][0].compare(ui->comboBox_class->currentText().toStdString().c_str()) == 0) {
      	  std::vector<int> indices;
      	  Eigen::Vector4f min_pt(s2d(labels[i][4]), s2d(labels[i][5]), s2d(labels[i][6]), 0);
      	  Eigen::Vector4f max_pt(s2d(labels[i][7]), s2d(labels[i][8]), s2d(labels[i][9]), 0);
      	  pcl::PointCloud<pcl::PointXYZI>::Ptr cluster(new pcl::PointCloud<pcl::PointXYZI>);
      	  pcl::getPointsInBox(*cloud, min_pt, max_pt, indices);
      	  pcl::copyPointCloud(*cloud, indices, *cluster);
	  if(cluster->size() >= 3) { // at lest 3 points for PCA
	    featureExtraction(cluster, false);
	    std::vector<Feature>::iterator it = features.end();
	    it--;
	    writeData(feature_file, it, 1);
	    n++;
	  }
      	}
      }
      feature_file.close();
      ui->label_show->setText("<font color=\"blue\">Feature extracted from "+QString::number(n)+" sample(s).</font>");
    } else {
      ui->label_show->setText("<font color=\"blue\">File without labels.</font>");
    }
    if(auto_next) {
      ui->listWidget_files->setCurrentRow(ui->listWidget_files->currentRow()+1);
    }
  }
  
  if(ui->comboBox_policy->currentIndex() == 2) {
    //@todo multiple files
    ui->label_show->setText("<font color=\"red\">Not available in the current version.</font>");
  }
}

void CloudViewer::backgroundButtonClicked() {
  if(!QDir("svm").exists()) {
    QDir().mkdir("svm");
  }
  
  if(file_labeled) {
    ui->label_show->setText("<font color=\"blue\">File with labels: unlabeled clusters are<br/>extracted as background samples.</font>");
  } else {
    ui->label_show->setText("<font color=\"blue\">File without labels: all clusters are<br/>extracted as background samples.</font>");
  }
  
  bool background;
  for(std::vector<Feature>::iterator it = features.begin(); it != features.end(); ++it) {
    feature_file.open("svm/background", std::fstream::in | std::fstream::out | std::fstream::app);
    background = true;
    
    if(file_labeled) {
      for(int i = 0; i < labels.size(); i++) {
	if(iou3d(it->min[0], it->min[1], it->min[2], it->max[0], it->max[1], it->max[2],
		 s2d(labels[i][4]), s2d(labels[i][5]), s2d(labels[i][6]), s2d(labels[i][7]), s2d(labels[i][8]), s2d(labels[i][9])) >= IOU_T) {
	  background = false;
	}
      }
    }
    
    if(background) {
      writeData(feature_file, it, -1);
      viewer->addCube(it->min[0], it->max[0], it->min[1], it->max[1], it->min[2], it->max[2], 0.0, 0.0, 1.0, "background_box_"+boost::to_string(it->id));
      viewer->setRepresentationToWireframeForAllActors();
      viewer->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_LINE_WIDTH, 4, "background_box_"+boost::to_string(it->id));
    }
    
    feature_file.close();
  }
}

void CloudViewer::trainButtonClicked() { // @todo use svm.h
  QProcess *process = new QProcess;
  process->start("bash", QStringList() << "-c" << "cat svm/background >> svm/"+ui->comboBox_class->currentText());
  process->waitForFinished();
  
  int exit_code = QProcess::execute("svm-easy svm/"+ui->comboBox_class->currentText());
  if(exit_code != 0) {
    ui->label_show->setText("<font color=\"red\">Training failed.</font>");
  } else {
    ui->label_show->setText("<font color=\"blue\">Training success.</font>");
  }
  QProcess::execute("bash", QStringList() << "-c" << "mv "+ui->comboBox_class->currentText()+".* svm/");
}

void CloudViewer::predictButtonClicked() { // @todo use svm.h
  QProcess *process = new QProcess;
  
  if(!QDir("svm").exists()) {
    QDir().mkdir("svm");
  }
  
  test_file.open("svm/test", std::fstream::in | std::fstream::out | std::fstream::trunc);
  for(std::vector<Feature>::iterator it = features.begin(); it != features.end(); ++it) {
    writeData(test_file, it, -1);
  }
  test_file.close();
  
  process->start("bash", QStringList() << "-c" << "svm-scale -r svm/"+ui->comboBox_class->currentText()+".range svm/test > svm/test.scale");
  process->waitForFinished();
  process->start("bash", QStringList() << "-c" << "svm-predict svm/test.scale svm/"+ui->comboBox_class->currentText()+".model svm/predict.output");
  process->waitForFinished();
    
  //viewer->removeAllShapes();
  std::ifstream file("svm/predict.output");
  std::string label;
  int n = 0;
  for(std::vector<Feature>::iterator it = features.begin(); it != features.end(); ++it) {
    std::getline(file, label);
    if(label.at(0) == '1') {
      viewer->addCube(it->min[0], it->max[0], it->min[1], it->max[1], it->min[2], it->max[2], 0.0, 1.0, 1.0, "predict_box_"+boost::to_string(it->id));
      viewer->setRepresentationToWireframeForAllActors();
      viewer->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_LINE_WIDTH, 4, "predict_box_"+boost::to_string(it->id));
      //pcl::PointXYZ pos(it->centroid[0], it->centroid[1], it->centroid[2]);
      //viewer->addText3D(boost::to_string(it->id), pos, 0.2, 1.0, 0.0, 0.0, "predict_id_"+boost::to_string(it->id));
      n++;
    }
  }
  ui->qvtkWidget->update();
  if(n == 0) {
    ui->label_show->setText("<font color=\"blue\">No "+ui->comboBox_class->currentText()+" / " +QString::number(features.size())+" clusters.</font>");
  } else {
    ui->label_show->setText("<font color=\"blue\">"+QString::number(n)+" "+ui->comboBox_class->currentText()+" / " +QString::number(features.size())+" clusters.</font>");
  }
  
  if(auto_next) {
    ui->listWidget_files->setCurrentRow(ui->listWidget_files->currentRow()+1);
  }
}

void CloudViewer::reloadButtonClicked() {
  if(ui->listWidget_files->currentRow() != -1) fileItemChanged();
}

void CloudViewer::adaptiveBoxChecked() {
  adaptive_clustering = (ui->checkBox_adaptive->isChecked()) ? true : false;
  if(ui->listWidget_files->currentRow() != -1) fileItemChanged();
}

void CloudViewer::intensityBoxChecked() {
  show_intensity = (ui->checkBox_intensity->isChecked()) ? true : false;
  if(ui->listWidget_files->currentRow() != -1) fileItemChanged();
}

void CloudViewer::nextBoxChecked() {
  auto_next = (ui->checkBox_next->isChecked()) ? true : false;
}

void CloudViewer::trunkBoxChecked() {
  trunk_show = (ui->checkBox_trunk->isChecked()) ? true : false;
  if(ui->listWidget_files->currentRow() != -1) fileItemChanged();
}

void CloudViewer::sliceBoxChecked() {
  slice_show = (ui->checkBox_slice->isChecked()) ? true : false;
  if(ui->listWidget_files->currentRow() != -1) fileItemChanged();
}

void CloudViewer::binsBoxChecked() {
  bins_show = (ui->checkBox_bins->isChecked()) ? true : false;
  if(ui->listWidget_files->currentRow() != -1) fileItemChanged();
}

void CloudViewer::fileItemChanged() {
  clustering(ui->listWidget_files->currentItem()->text().toStdString());
  
  file_labeled = false;
  current_label_path = QFileInfo(ui->listWidget_files->currentItem()->text()).absolutePath()+"/label";
  if(QDir(current_label_path).exists()) {
    QDirIterator dir_it(current_label_path, QDirIterator::Subdirectories);
    QString file;
    while(dir_it.hasNext()) {
      file = dir_it.next();
      if(QFileInfo(file).fileName().toStdString().find('~') == std::string::npos &&
	 QFileInfo(file).completeBaseName() == QFileInfo(ui->listWidget_files->currentItem()->text()).completeBaseName()) {
	label_file.open((current_label_path+"/"+QFileInfo(ui->listWidget_files->currentItem()->text()).completeBaseName()+".txt").toStdString().c_str(), std::fstream::in);
	labels.clear();
	std::string line;
	while(std::getline(label_file, line)) {
	  std::vector<std::string> params;
	  boost::split(params, line, boost::is_any_of(" "));
	  labels.push_back(params);
	  pcl::PointXYZ pos(s2d(params[1]), s2d(params[2]), s2d(params[9]));
	  double r, g, b;
	  if(params[0].compare("pedestrian") == 0) {r=1; g=0; b=0;}
	  if(params[0].compare("group") == 0)      {r=0; g=1; b=0;}
	  if(params[0].compare("wheelchair") == 0) {r=0; g=0; b=1;}
	  if(params[0].compare("cyclist") == 0)    {r=1; g=1; b=0;}
	  if(params[0].compare("car") == 0)        {r=0; g=1; b=1;}
	  viewer->addText3D(params[0], pos, 0.2, r, g, b, "labeled_text_"+params[1]);
	  viewer->addCube(s2d(params[4]), s2d(params[7]),
			  s2d(params[5]), s2d(params[8]),
			  s2d(params[6]), s2d(params[9]),
			  r, g, b, "labeled_box_"+boost::to_string(params[1]));
	  viewer->setRepresentationToWireframeForAllActors();
	  viewer->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_LINE_WIDTH, 4, "labeled_box_"+boost::to_string(params[1]));
	  ui->qvtkWidget->update();
	}
	label_file.close();
	file_labeled = true;
	ui->label_show->setText("<font color=\"blue\">File labeled.</font>");
	break;
      }
    }
  }
  if(file_labeled == false) {
    ui->label_show->setText("<font color=\"blue\">File without labels.</font>");
  }
}

void CloudViewer::clustering(std::string file_name) {  
  cloud.reset(new pcl::PointCloud<pcl::PointXYZI>);
  
  // Load point cloud.
  if(pcl::io::loadPCDFile(file_name, *cloud) != 0) {
    PCL_ERROR("Error reading point cloud %s\n", file_name.c_str());
    return;
  }
  
  if(!cloud->is_dense) {
    PCL_WARN("Cloud is not dense! Non finite points will be removed\n");
    std::vector<int> index;
    pcl::removeNaNFromPointCloud(*cloud, *cloud, index);
  }
  
  viewer->removeAllPointClouds();
  viewer->removeAllShapes();
  features.clear();

  pcl::IndicesPtr indices(new std::vector<int>);
  
  // [OPTIONAL] Removing ground plane and top surface.
  if(remove_top_bottom) {
    pcl::PassThrough<pcl::PointXYZI> pt;
    pt.setInputCloud(cloud);
    pt.setFilterFieldName("z");
    pt.setFilterLimits(-ui->bottominterval->text().toDouble(), ui->topinterval->text().toDouble());
    pt.filter(*indices);
    pcl::copyPointCloud(*cloud, *indices, *cloud);
  }
  
  // [OPTIONAL] Removing outliers.
  if(remove_outliers) {
    std::cerr << "Points before removing outliers: " << cloud->size() << std::endl;
    indices.reset(new std::vector<int>);
    pcl::StatisticalOutlierRemoval<pcl::PointXYZI> sor;
    sor.setInputCloud(cloud);
    sor.setMeanK(50);
    sor.setStddevMulThresh(1.0);
    sor.filter(*indices);
    pcl::copyPointCloud(*cloud, *indices, *cloud);
    std::cerr << "Points after removing outliers: " << cloud->size() << std::endl;
  }
  
  // [OPTIONAL] Downsampling.
  if(down_sampling) {
    std::cerr << "Points before downsampling: " << cloud->size() << std::endl;
    pcl::PointCloud<pcl::PointXYZI>::Ptr pc(new pcl::PointCloud<pcl::PointXYZI>);
    pcl::VoxelGrid<pcl::PointXYZI> vg;
    vg.setInputCloud(cloud);
    vg.setLeafSize(0.06, 0.06, 0.06);
    vg.filter(*pc);
    pcl::copyPointCloud(*pc, *cloud);
    std::cerr << "Points after downsampling: " << cloud->size() << std::endl;
  }
  
  // [OPTIONAL] Removing planes.
  if(remove_planes) {
    std::cerr << "Points before removing planes: " << cloud->size() << std::endl;
    pcl::SACSegmentation<pcl::PointXYZI> seg;
    pcl::ExtractIndices<pcl::PointXYZI> ei;
    pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
    pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
    
    seg.setOptimizeCoefficients(true);
    seg.setModelType(pcl::SACMODEL_PLANE);
    seg.setMethodType(pcl::SAC_RANSAC);
    seg.setMaxIterations(100); // default:50
    seg.setDistanceThreshold(0.2); // default:0
    seg.setInputCloud(cloud);
    seg.segment(*inliers, *coefficients);
    
    indices.reset(new std::vector<int>);
    ei.setInputCloud(cloud);
    ei.setIndices(inliers);
    ei.setNegative(true);
    ei.filter(*indices);
    pcl::copyPointCloud(*cloud, *indices, *cloud);
    std::cerr << "Points after removing planes: " << cloud->size() << std::endl;
  }
  
  // [MANDATORY] Euclidean clustering.
  if(adaptive_clustering) {
    const int nested_regions = 14; // @todo channge it to a adjustable parameter
    const int zone[14] = {2,3,3,3,3,3,3,2,3,3,3,3,3,3}; // Velodyne VLP-16
    boost::array<std::vector<int>, nested_regions> regions_indices;
    
    for(int i = 0; i < cloud->size(); i++) {
      float range = 0.0;
      for(int j = 0; j < nested_regions; j++) {
	float distance2 =
	  cloud->points[i].x*cloud->points[i].x +
	  cloud->points[i].y*cloud->points[i].y +
	  cloud->points[i].z*cloud->points[i].z;
	if(distance2 > range*range && distance2 <= (range+zone[j])*(range+zone[j])) {
	  regions_indices[j].push_back(i);
	  break;
	}
	range += zone[j];
      }
    }
    
    float tolerance = 0.0;
    for(int i = 0; i < nested_regions; i++) {
      tolerance += 0.1;
      if(regions_indices[i].size() > ui->minclustersize->text().toDouble()) {
	boost::shared_ptr<std::vector<int> > regions_indices_ptr(new std::vector<int>(regions_indices[i]));
	pcl::search::KdTree<pcl::PointXYZI>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZI>);
	tree->setInputCloud(cloud, regions_indices_ptr);
	
	std::vector<pcl::PointIndices> cluster_indices;
	pcl::EuclideanClusterExtraction<pcl::PointXYZI> ec;
	ec.setClusterTolerance(tolerance);
	ec.setMinClusterSize(ui->minclustersize->text().toDouble());
	ec.setMaxClusterSize(ui->maxclustersize->text().toDouble());
	ec.setSearchMethod(tree);
	ec.setInputCloud(cloud);
	ec.setIndices(regions_indices_ptr);
	ec.extract(cluster_indices);
	
	for(std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin(); it != cluster_indices.end(); ++it) {
	  pcl::PointCloud<pcl::PointXYZI>::Ptr cluster(new pcl::PointCloud<pcl::PointXYZI>);
	  for(std::vector<int>::const_iterator pit = it->indices.begin(); pit != it->indices.end(); ++pit) {
	    cluster->points.push_back(cloud->points[*pit]);
	  }
	  cluster->width = cluster->size();
	  cluster->height = 1;
	  cluster->is_dense = true;
	  featureExtraction(cluster, true);
	}
      }
    }
  } else {
    pcl::search::KdTree<pcl::PointXYZI>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZI>);
    tree->setInputCloud(cloud);
    
    std::vector<pcl::PointIndices> cluster_indices;
    pcl::EuclideanClusterExtraction<pcl::PointXYZI> ec;
    ec.setClusterTolerance(ui->clustertolerance->text().toDouble());
    ec.setMinClusterSize(ui->minclustersize->text().toDouble());
    ec.setMaxClusterSize(ui->maxclustersize->text().toDouble());
    ec.setSearchMethod(tree);
    ec.setInputCloud(cloud);
    ec.extract(cluster_indices);
    
    for(std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin(); it != cluster_indices.end(); ++it) {
      pcl::PointCloud<pcl::PointXYZI>::Ptr cluster(new pcl::PointCloud<pcl::PointXYZI>);
      for(std::vector<int>::const_iterator pit = it->indices.begin(); pit != it->indices.end(); ++pit) {
	cluster->points.push_back(cloud->points[*pit]);
      }
      cluster->width = cluster->size();
      cluster->height = 1;
      cluster->is_dense = true;
      featureExtraction(cluster, true);
    }
  }
  
  if(!trunk_show && !slice_show && !bins_show) {
    if(!show_intensity) {
      viewer->addPointCloud<pcl::PointXYZI>(cloud);
      viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3);
    } else {
      pcl::visualization::PointCloudColorHandlerGenericField<pcl::PointXYZI> handler(cloud, "intensity");
      viewer->addPointCloud<pcl::PointXYZI>(cloud, handler);
    }
    viewer->addCoordinateSystem();
    ui->qvtkWidget->update();
  }
}

/* *** Feature Extraction ***
 * f1 (1d): the number of points included in a cluster.
 * f2 (1d): the minimum distance of the cluster to the sensor.
 * => f1 and f2 should be used in pairs, since f1 varies with f2 changes.
 * f3 (6d): 3D covariance matrix of the cluster.
 * f4 (6d): the normalized moment of inertia tensor.
 * => Since both f3 and f4 are symmetric, we only use 6 elements from each as features.
 * f5 (9d): 2D covariance matrix in 3 zones, which are the upper half, and the left and right lower halves.
 * f6 (98d): The normalized 2D histogram for the main plane, 14 × 7 bins.
 * f7 (45d): The normalized 2D histogram for the secondary plane, 9 × 5 bins.
 * f8 (30d): Slice feature for the cluster.
 * f9 (27d): Intensity.
 */

void CloudViewer::featureExtraction(pcl::PointCloud<pcl::PointXYZI>::Ptr pc, bool visual) {
  Eigen::Vector4f centroid, min, max;
  pcl::compute3DCentroid(*pc, centroid);
  pcl::getMinMax3D(*pc, min, max);
  
  Feature f;
  f.id = features.size();
  f.centroid = centroid;
  f.min = min;
  f.max = max;
  // f1: Number of points included the cluster.
  f.number_points = pc->size();
  // f2: The minimum distance to the cluster.
  f.min_distance = FLT_MAX;
  float d2; //squared Euclidean distance
  for(int i = 0; i < pc->size(); ++i) {
    d2 = pc->points[i].x*pc->points[i].x + pc->points[i].y*pc->points[i].y + pc->points[i].z*pc->points[i].z;
    if(f.min_distance > d2)
      f.min_distance = d2;
  }
  //f.min_distance = sqrt(f.min_distance);
  
  pcl::PCA<pcl::PointXYZI> pca;
  pcl::PointCloud<pcl::PointXYZI>::Ptr pc_projected(new pcl::PointCloud<pcl::PointXYZI>);
  pca.setInputCloud(pc);
  pca.project(*pc, *pc_projected);
  // f3: 3D covariance matrix of the cluster.
  pcl::computeCovarianceMatrixNormalized(*pc_projected, centroid, f.covariance_3d);
  // f4: The normalized moment of inertia tensor.
  computeMomentOfInertiaTensorNormalized(*pc_projected, f.moment_3d);  
  // Navarro et al. assume that a pedestrian is in an upright position, so the principal component e1 is expected to be vertically aligned with the person's body.
  pcl::PointCloud<pcl::PointXYZ>::Ptr main_plane(new pcl::PointCloud<pcl::PointXYZ>), secondary_plane(new pcl::PointCloud<pcl::PointXYZ>);
  computeProjectedPlane(pc, f.id, pca.getEigenVectors(), 2, centroid, main_plane);
  computeProjectedPlane(pc, f.id, pca.getEigenVectors(), 1, centroid, secondary_plane);
  // f5: 2D covariance matrix in 3 zones, which are the upper half, and the left and right lower halves.
  compute3ZoneCovarianceMatrix(main_plane, f.id, pca.getMean(), f.partial_covariance_2d);
  // f6 and f7
  computeHistogramNormalized(main_plane, f.id, 7, 14, f.histogram_main_2d);
  computeHistogramNormalized(secondary_plane, f.id, 5, 9, f.histogram_second_2d);
  // f8
  computeSlice(pc, 10, f);
  // f9
  computeIntensity(pc, 25, f);
  
  features.push_back(f);
  
  if(trunk_show) {
    /* Print mean, eigen values and eigen vectors. */
    std::cerr << "****** Cluster ID " << f.id << " ******" << std::endl;
    std::cerr << "Centroid:\n" << centroid << std::endl;
    std::cerr << "Min:\n" << min << std::endl;
    std::cerr << "Max:\n" << max << std::endl;
    std::cerr << "Mean:\n" << pca.getMean() << std::endl;
    std::cerr << "EigenValues:\n" << pca.getEigenValues() << std::endl;
    std::cerr << "EigenVectors:\n" << pca.getEigenVectors() << std::endl;
    /* Eigen vectors visualization. */
    pcl::PointXYZ eigen_center(pca.getMean()(0), pca.getMean()(1), pca.getMean()(2));
    pcl::PointXYZ eigen_x(pca.getEigenVectors()(0,0) + pca.getMean()(0),
  			  pca.getEigenVectors()(1,0) + pca.getMean()(1),
  			  pca.getEigenVectors()(2,0) + pca.getMean()(2));
    pcl::PointXYZ eigen_y(pca.getEigenVectors()(0,1) + pca.getMean()(0),
  			  pca.getEigenVectors()(1,1) + pca.getMean()(1),
  			  pca.getEigenVectors()(2,1) + pca.getMean()(2));
    pcl::PointXYZ eigen_z(pca.getEigenVectors()(0,2) + pca.getMean()(0),
  			  pca.getEigenVectors()(1,2) + pca.getMean()(1),
  			  pca.getEigenVectors()(2,2) + pca.getMean()(2));
    viewer->addLine(eigen_center, eigen_x, 1.0, 0.0, 0.0, "eigen_x_"+boost::to_string(f.id));
    viewer->addLine(eigen_center, eigen_y, 0.0, 1.0, 0.0, "eigen_y_"+boost::to_string(f.id));
    viewer->addLine(eigen_center, eigen_z, 0.0, 0.0, 1.0, "eigen_z_"+boost::to_string(f.id));
  }
  
  // Draw cluster box.
  if(visual) {
    double r = std::max(0.3, (double)rand()/RAND_MAX);
    double g = std::max(0.3, (double)rand()/RAND_MAX);
    double b = std::max(0.3, (double)rand()/RAND_MAX);
    viewer->addCube(min[0], max[0], min[1], max[1], min[2], max[2], r, g, b, "box_"+boost::to_string(f.id));
    viewer->setRepresentationToWireframeForAllActors();
    pcl::PointXYZ pos(centroid[0], centroid[1], centroid[2]);
    viewer->addText3D(boost::to_string(f.id), pos, 0.3, r, g, b, "id_"+boost::to_string(f.id));
  }
}

void CloudViewer::computeMomentOfInertiaTensorNormalized(pcl::PointCloud<pcl::PointXYZI> &pc, Eigen::Matrix3f &moment_3d) {
  moment_3d.setZero();
  for(size_t i = 0; i < pc.size(); ++i) {
    moment_3d(0,0) += pc[i].y*pc[i].y+pc[i].z*pc[i].z;
    moment_3d(0,1) -= pc[i].x*pc[i].y;
    moment_3d(0,2) -= pc[i].x*pc[i].z;
    moment_3d(1,1) += pc[i].x*pc[i].x+pc[i].z*pc[i].z;
    moment_3d(1,2) -= pc[i].y*pc[i].z;
    moment_3d(2,2) += pc[i].x*pc[i].x+pc[i].y*pc[i].y;
  }
  moment_3d(1, 0) = moment_3d(0, 1);
  moment_3d(2, 0) = moment_3d(0, 2);
  moment_3d(2, 1) = moment_3d(1, 2);
}

/* Main plane is formed from the maximum and middle eigenvectors.
 * Secondary plane is formed from the middle and minimum eigenvectors.
 */
void CloudViewer::computeProjectedPlane(pcl::PointCloud<pcl::PointXYZI>::Ptr pc, int id, Eigen::Matrix3f &eigenvectors, int axe, Eigen::Vector4f &centroid, pcl::PointCloud<pcl::PointXYZ>::Ptr plane) {
  Eigen::Vector4f coefficients;
  coefficients[0] = eigenvectors(0,axe);
  coefficients[1] = eigenvectors(1,axe);
  coefficients[2] = eigenvectors(2,axe);
  coefficients[3] = 0;
  coefficients[3] = -1 * coefficients.dot(centroid);
  for(size_t i = 0; i < pc->size(); ++i) {
    float distance_to_plane =
      coefficients[0] * pc->points[i].x +
      coefficients[1] * pc->points[i].y +
      coefficients[2] * pc->points[i].z +
      coefficients[3];
    pcl::PointXYZ p;
    p.x = pc->points[i].x - distance_to_plane * coefficients[0];
    p.y = pc->points[i].y - distance_to_plane * coefficients[1];
    p.z = pc->points[i].z - distance_to_plane * coefficients[2];
    plane->points.push_back(p);
  }
  
  if(bins_show) {
    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> rgb(plane, axe==0?255:0, axe==1?255:0, axe==2?255:0);
    viewer->addPointCloud(plane, rgb, "plane_"+boost::to_string(axe)+"_"+boost::to_string(id));
    ui->qvtkWidget->update();
  }
}

/* Upper half, and the left and right lower halves of a pedestrian. */
void CloudViewer::compute3ZoneCovarianceMatrix(pcl::PointCloud<pcl::PointXYZ>::Ptr plane, int id, Eigen::Vector4f &mean, float *partial_covariance_2d) {
  pcl::PointCloud<pcl::PointXYZ>::Ptr zone_decomposed[3];
  for(int i = 0; i < 3; i++)
    zone_decomposed[i].reset(new pcl::PointCloud<pcl::PointXYZ>);
  for(size_t i = 0; i < plane->size(); ++i) {
    if(plane->points[i].z >= mean(2)) { // upper half
      zone_decomposed[0]->points.push_back(plane->points[i]);
    } else {
      if(plane->points[i].y >= mean(1)) // left lower half
	zone_decomposed[1]->points.push_back(plane->points[i]);
      else // right lower half
	zone_decomposed[2]->points.push_back(plane->points[i]);
    }
  }
  
  Eigen::Matrix3f covariance;
  Eigen::Vector4f centroid;
  for(int i = 0; i < 3; i++) {
    pcl::compute3DCentroid(*zone_decomposed[i], centroid);
    pcl::computeCovarianceMatrix(*zone_decomposed[i], centroid, covariance);
    partial_covariance_2d[i*3+0] = covariance(0,0);
    partial_covariance_2d[i*3+1] = covariance(0,1);
    partial_covariance_2d[i*3+2] = covariance(1,1);
  }
  
  if(trunk_show) {
    for(int i = 0; i < 3; i++) {
      pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> rgb(zone_decomposed[i], i==0?255:98, i==1?255:98, i==2?255:98);
      viewer->addPointCloud(zone_decomposed[i], rgb, "zone_decomposed_"+boost::to_string(i)+"_"+boost::to_string(id));
      ui->qvtkWidget->update();
    }
  }
}

void CloudViewer::computeHistogramNormalized(pcl::PointCloud<pcl::PointXYZ>::Ptr pc, int id, int horiz_bins, int verti_bins, float *histogram) {
  Eigen::Vector4f min, max, min_box, max_box;
  pcl::getMinMax3D(*pc, min, max);
  float horiz_itv, verti_itv;
  horiz_itv = (max[0]-min[0]>max[1]-min[1]) ? (max[0]-min[0])/horiz_bins : (max[1]-min[1])/horiz_bins;
  verti_itv = (max[2] - min[2])/verti_bins;
  
  for(int i = 0; i < horiz_bins; ++i) {
    for(int j = 0; j < verti_bins; ++j) {
      if(max[0]-min[0] > max[1]-min[1]) {
	min_box << min[0]+horiz_itv*i, min[1], min[2]+verti_itv*j, 0;
	max_box << min[0]+horiz_itv*(i+1), max[1], min[2]+verti_itv*(j+1), 0;
      } else {
	min_box << min[0], min[1]+horiz_itv*i, min[2]+verti_itv*j, 0;
	max_box << max[0], min[1]+horiz_itv*(i+1), min[2]+verti_itv*(j+1), 0;
      }
      std::vector<int> indices;
      pcl::getPointsInBox(*pc, min_box, max_box, indices);
      histogram[i*verti_bins+j] = (float)indices.size() / (float)pc->size();
      if(bins_show) {
      	pcl::getMinMax3D(*pc, indices, min_box, max_box);
      	viewer->addCube(min_box[0], max_box[0], min_box[1], max_box[1], min_box[2], max_box[2],
      			std::max(0.3,(double)rand()/RAND_MAX), std::max(0.3,(double)rand()/RAND_MAX), std::max(0.3,(double)rand()/RAND_MAX),
      			"histogram_"+boost::to_string(i)+"_"+boost::to_string(j)+"_"+boost::to_string(horiz_bins*verti_bins)+"_"+boost::to_string(id));
	viewer->setRepresentationToWireframeForAllActors();
      	ui->qvtkWidget->update();
      }
    }
  }
}

void CloudViewer::computeSlice(pcl::PointCloud<pcl::PointXYZI>::Ptr pc, int n, Feature &f) {
  pcl::PointCloud<pcl::PointXYZI>::Ptr pc_blocks[n];
  float itv = (f.max[2] - f.min[2]) / n;
  if(itv > 0) {
    for(int i = 0; i < n; ++i) {
      pc_blocks[i].reset(new pcl::PointCloud<pcl::PointXYZI>);
    }
    for(unsigned int i = 0, j; i < pc->size(); ++i) {
      j = std::min((n-1), (int)((pc->points[i].z - f.min[2]) / itv));
      pc_blocks[j]->points.push_back(pc->points[i]);
    }
    
    Eigen::Vector4f min, max, centroid;
    for(int i = 0; i < n; ++i) {
      if(pc_blocks[i]->size() > 0) {
	pcl::getMinMax3D(*pc_blocks[i], min, max);
	pcl::compute3DCentroid(*pc_blocks[i], centroid);
      } else {
	min.setZero();
	max.setZero();
	centroid.setZero();
      }
      f.slice[i*2] = max[0] - min[0];
      f.slice[i*2+1] = max[1] - min[1];
      f.slice[i+20] = centroid[0]*centroid[0] + centroid[1]*centroid[1] + centroid[2]*centroid[2];
      
      if(slice_show) {
	viewer->addPointCloud<pcl::PointXYZI>(pc_blocks[i], "block_"+boost::to_string(i)+"_"+boost::to_string(f.id));
	viewer->addCube(min[0], max[0], min[1], max[1], min[2], max[2], 1.0, 0.27, 0.0, "slice_"+boost::to_string(i)+"_"+boost::to_string(f.id));
	viewer->setRepresentationToWireframeForAllActors();
	ui->qvtkWidget->update();
      }
    }
  } else {
    for(int i = 0; i < 30; ++i)
      f.slice[i] = 0;
  }
}

void CloudViewer::computeIntensity(pcl::PointCloud<pcl::PointXYZI>::Ptr pc, int bins, Feature &f) {
  float sum = 0, mean = 0, sum_dev = 0;
  float min = FLT_MAX, max = -FLT_MAX;
  for(int i = 0; i < 27; ++i)
    f.intensity[i] = 0;
  
  for(size_t i = 0; i < pc->size(); ++i) {
    sum += pc->points[i].intensity;
    min = std::min(min, pc->points[i].intensity);
    max = std::max(max, pc->points[i].intensity);
  }
  mean = sum / pc->size();
  
  for(size_t i = 0; i < pc->size(); ++i) {
    sum_dev += (pc->points[i].intensity-mean)*(pc->points[i].intensity-mean);
    int ii = std::min(float(bins-1), std::floor((pc->points[i].intensity-min)/((max-min)/bins)));
    f.intensity[ii]++;
  }
  f.intensity[25] = sqrt(sum_dev/pc->size());
  f.intensity[26] = mean;
}
