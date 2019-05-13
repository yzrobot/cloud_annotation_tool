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
#include <pcl/filters/passthrough.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/common/centroid.h>
#include <pcl/common/common.h>
#include <pcl/common/pca.h>
#include <pcl/visualization/pcl_visualizer.h>
// Visualization Toolkit (VTK)
#include <vtkRenderWindow.h>

typedef struct feature {
  // for labeling and visualization
  int id;
  Eigen::Vector4f centroid;
  Eigen::Vector4f min;
  Eigen::Vector4f max;
  // for training and classification
  int number_points;
  float min_distance;
  Eigen::Matrix3f covariance_3d;
  Eigen::Matrix3f moment_3d;
  float partial_covariance_2d[9];
  float histogram_main_2d[98];
  float histogram_second_2d[45];
  float slice[30];
  float intensity[27];
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
    void extractButtonClicked();
    void trainButtonClicked();
    void predictButtonClicked();
    void reloadButtonClicked();
    
    void adaptiveBoxChecked();
    void intensityBoxChecked();
    void nextBoxChecked();
    void trunkBoxChecked();
    void sliceBoxChecked();
    void binsBoxChecked();
    
    void fileItemChanged();
    
 protected:
    boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer;
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud;
    
    void clustering(std::string file_name);
    void featureExtraction(pcl::PointCloud<pcl::PointXYZI>::Ptr cloud, bool visual);
    void computeMomentOfInertiaTensorNormalized(pcl::PointCloud<pcl::PointXYZI> &cloud, Eigen::Matrix3f &tensor);
    void computeProjectedPlane(pcl::PointCloud<pcl::PointXYZI>::Ptr cloud, int id, Eigen::Matrix3f &eigenvectors, int axe, Eigen::Vector4f &centroid, pcl::PointCloud<pcl::PointXYZ>::Ptr plane);
    void compute3ZoneCovarianceMatrix(pcl::PointCloud<pcl::PointXYZ>::Ptr plane, int id, Eigen::Vector4f &mean, float *partial_covariance_2d);
    void computeHistogramNormalized(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, int id, int horiz_bins, int verti_bins, float *histogram);
    void computeSlice(pcl::PointCloud<pcl::PointXYZI>::Ptr cloud, int n, Feature &f);
    void computeIntensity(pcl::PointCloud<pcl::PointXYZI>::Ptr cloud, int bins, Feature &f);
    
 private:
    Ui::CloudViewer *ui;
    QString load_file_path, current_label_path;
    bool file_labeled;
    bool adaptive_clustering, show_intensity, auto_next;
    bool trunk_show, slice_show, bins_show;
    
    bool remove_top_bottom;
    bool down_sampling;
    bool remove_outliers;
    bool remove_planes;
    
    std::vector<Feature> features;
    std::vector< std::vector<std::string> > labels;
    std::fstream label_file, feature_file, test_file, output_file;
};

#endif // VIEWER_H
