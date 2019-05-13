#ifndef VIEWER_H
#define VIEWER_H

// C++
#include <iostream>
// Qt
#include <QMainWindow>
#include <QFileDialog>
#include <QListWidgetItem>
#include <QProcess>
#include <QDirIterator>
// Point Cloud Library
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/common/centroid.h>
#include <pcl/common/common.h>
#include <pcl/visualization/pcl_visualizer.h>
// Visualization Toolkit (VTK)
#include <vtkRenderWindow.h>

typedef struct feature {
  int id;
  Eigen::Vector4f centroid;
  Eigen::Vector4f min;
  Eigen::Vector4f max;
} Feature;

namespace Ui {
  class CloudViewer;
}

class CloudViewer: public QMainWindow {
  Q_OBJECT
  
 public:
  explicit CloudViewer(QWidget *parent = 0);
  ~CloudViewer();
  
  public slots:
    void loadButtonClicked();
    void labelButtonClicked();
    void reloadButtonClicked();
    void nextBoxChecked();
    void fileItemChanged();
    
 protected:
    boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer;
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, cloud_tmp;
    
    void clustering(std::string file_name);
    void featureExtraction(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud);
    
 private:
    Ui::CloudViewer *ui;
    QString load_file_path, current_label_path;
    bool file_labeled;
    bool auto_next;
    
    bool remove_top_bottom;
    bool down_sampling;
    bool remove_outliers;
    bool remove_planes;
    
    std::vector<Feature> features;
    std::vector< std::vector<std::string> > labels;
    std::fstream label_file;
}; 

#endif // VIEWER_H
