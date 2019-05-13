#include "viewer.h"
#include "build/ui_viewer.h"

CloudViewer::CloudViewer(QWidget *parent):
  QMainWindow(parent),
  ui(new Ui::CloudViewer) {
  ui->setupUi(this);
  this->setWindowIcon(QIcon(":/images/lcas_logo.png"));
  this->setWindowTitle("L-CAS Cloud Annotation Tool");
  load_file_path = std::getenv("HOME");
  
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
  connect(ui->pushButton_reload, SIGNAL(clicked()), this, SLOT(reloadButtonClicked()));
  connect(ui->checkBox_next, SIGNAL(stateChanged(int)), this, SLOT(nextBoxChecked()));
  connect(ui->listWidget_files, SIGNAL(itemSelectionChanged()), this, SLOT(fileItemChanged()));
}

CloudViewer::~CloudViewer() {
  delete ui;
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
  
  if(QDir(current_label_path).exists() == false) {
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
	if(line_in.substr(line_in.find(" ")+1, line_in.length()-line_in.find(" ")-3).compare(line_to) == 0) {
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

void CloudViewer::reloadButtonClicked() {
  if(ui->listWidget_files->currentRow() != -1) {
    fileItemChanged();
  }
}

void CloudViewer::nextBoxChecked() {
  auto_next = (ui->checkBox_next->isChecked()) ? true : false;
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
	  pcl::PointXYZ pos(atof(params[1].c_str()), atof(params[2].c_str()), atof(params[9].c_str()));
	  double r, g, b;
	  if(params[0].compare("pedestrian") == 0) {r=1; g=0; b=0;}
	  if(params[0].compare("group") == 0)      {r=0; g=1; b=0;}
	  if(params[0].compare("wheelchair") == 0) {r=0; g=0; b=1;}
	  if(params[0].compare("cyclist") == 0)    {r=1; g=1; b=0;}
	  if(params[0].compare("car") == 0)        {r=0; g=1; b=1;}
	  viewer->addText3D(params[0], pos, 0.2, r, g, b, "labeled_text_"+params[1]);
	  viewer->addCube(atof(params[4].c_str()), atof(params[7].c_str()),
			  atof(params[5].c_str()), atof(params[8].c_str()),
			  atof(params[6].c_str()), atof(params[9].c_str()),
			  r, g, b, "labeled_box_"+boost::to_string(params[1]));
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
  cloud.reset(new pcl::PointCloud<pcl::PointXYZ>);
  cloud_tmp.reset(new pcl::PointCloud<pcl::PointXYZ>);
  
  // Load point cloud.
  if(pcl::io::loadPCDFile(file_name, *cloud_tmp) != 0) {
    PCL_ERROR("Error reading point cloud %s\n", file_name.c_str());
    return;
  }

  if(cloud_tmp->is_dense) {
    pcl::copyPointCloud(*cloud_tmp, *cloud);
  } else {
    PCL_WARN("Cloud is not dense! Non finite points will be removed\n");
    std::vector<int> vec;
    pcl::removeNaNFromPointCloud(*cloud_tmp, *cloud, vec);
  }
  
  // [OPTIONAL] Removing ground plane and top surface.
  if(remove_top_bottom) {
    Eigen::Vector4f min, max;
    pcl::getMinMax3D(*cloud, min, max);
    std::vector<int> indices;
    for(int i = 0; i < cloud->size(); ++i) {
      if(cloud->points[i].z-min[2] >= ui->bottominterval->text().toDouble() && max[2]-cloud->points[i].z >= ui->topinterval->text().toDouble()) {
	indices.push_back(i);
      }
    }
    pcl::copyPointCloud(*cloud, indices, *cloud);
  }
  
  // [OPTIONAL] Downsampling.
  if(down_sampling) {
    std::cerr << "Points before downsampling: " << cloud->size() << std::endl;
    cloud_tmp.reset(new pcl::PointCloud<pcl::PointXYZ>);
    
    pcl::VoxelGrid<pcl::PointXYZ> vg;
    vg.setInputCloud(cloud);
    vg.setLeafSize(0.06, 0.06, 0.06);
    vg.filter(*cloud_tmp);
    
    pcl::copyPointCloud(*cloud_tmp, *cloud);
    std::cerr << "Points after downsampling: " << cloud->size() << std::endl;
  }

  // [OPTIONAL] Removing outliers.
  if(remove_outliers) {
    std::cerr << "Points before removing outliers: " << cloud->size() << std::endl;
    cloud_tmp.reset(new pcl::PointCloud<pcl::PointXYZ>);
    
    pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sor;
    sor.setInputCloud(cloud);
    sor.setMeanK(50);
    sor.setStddevMulThresh(1.0);
    sor.filter(*cloud_tmp);
    
    pcl::copyPointCloud(*cloud_tmp, *cloud);
    std::cerr << "Points after removing outliers: " << cloud->size() << std::endl;
  }
  
  // [OPTIONAL] Removing planes.
  if(remove_planes) {
    std::cerr << "Points before removing planes: " << cloud->size() << std::endl;
    cloud_tmp.reset(new pcl::PointCloud<pcl::PointXYZ>);
    
    pcl::SACSegmentation<pcl::PointXYZ> seg;
    pcl::ExtractIndices<pcl::PointXYZ> ei;
    pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
    pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
    
    seg.setOptimizeCoefficients(true);
    seg.setModelType(pcl::SACMODEL_PLANE);
    seg.setMethodType(pcl::SAC_RANSAC);
    seg.setMaxIterations(100); // default:50
    seg.setDistanceThreshold(0.02); // default:0
    seg.setInputCloud(cloud);
    seg.segment(*inliers, *coefficients);
    
    ei.setInputCloud(cloud);
    ei.setIndices(inliers);
    ei.setNegative(true);
    ei.filter(*cloud_tmp);
    
    pcl::copyPointCloud(*cloud_tmp, *cloud);
    std::cerr << "Points after removing planes: " << cloud->size() << std::endl;
  }
  
  // [MANDATORY] Euclidean clustering.
  pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);
  tree->setInputCloud(cloud);
  
  std::vector<pcl::PointIndices> cluster_indices;
  pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
  ec.setClusterTolerance(ui->clustertolerance->text().toDouble());
  ec.setMinClusterSize(ui->minclustersize->text().toDouble());
  ec.setMaxClusterSize(ui->maxclustersize->text().toDouble());
  ec.setSearchMethod(tree);
  ec.setInputCloud(cloud);
  ec.extract(cluster_indices);
  
  viewer->removeAllPointClouds();
  viewer->removeAllShapes();
  features.clear();
  for(std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin(); it != cluster_indices.end(); ++it) {
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_cluster(new pcl::PointCloud<pcl::PointXYZ>);
    for(std::vector<int>::const_iterator pit = it->indices.begin(); pit != it->indices.end(); ++pit) {
      cloud_cluster->points.push_back(cloud->points[*pit]);
    }
    cloud_cluster->width = cloud_cluster->size();
    cloud_cluster->height = 1;
    cloud_cluster->is_dense = true;
    featureExtraction(cloud_cluster);
  }
  viewer->addPointCloud(cloud, "cloud");
  viewer->addCoordinateSystem();
  ui->qvtkWidget->update();
}

void CloudViewer::featureExtraction(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud) {
  Eigen::Vector4f centroid, min, max;
  pcl::compute3DCentroid(*cloud, centroid);
  pcl::getMinMax3D(*cloud, min, max);
  
  if(max[0]-min[0]<ui->minlwh->text().toDouble() || max[0]-min[0]>ui->maxlwh->text().toDouble() ||
     max[1]-min[1]<ui->minlwh->text().toDouble() || max[1]-min[1]>ui->maxlwh->text().toDouble() ||
     max[2]-min[2]<ui->minlwh->text().toDouble() || max[2]-min[2]>ui->maxlwh->text().toDouble() ||
     min[2]>ui->distancetoground->text().toDouble()) {
    return;
  }
  
  int id = features.size();
  
  Feature f;
  f.id = id;
  f.centroid = centroid;
  f.min = min;
  f.max = max;
  features.push_back(f);
  
  double r = (std::max)(0.3, (double)rand()/RAND_MAX);
  double g = (std::max)(0.3, (double)rand()/RAND_MAX);
  double b = (std::max)(0.3, (double)rand()/RAND_MAX);
  viewer->addCube(min[0], max[0], min[1], max[1], min[2], max[2], r, g, b, "box_"+boost::to_string(id));
  pcl::PointXYZ pos(centroid[0], centroid[1], centroid[2]);
  viewer->addText3D(boost::to_string(id), pos, 0.3, r, g, b, "id_"+boost::to_string(id));
}
