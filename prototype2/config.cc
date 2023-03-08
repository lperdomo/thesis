//
// Created by phi on 03/07/18.
//

#include "config.h"

namespace prototype2 {

Config::Config(int argc, char **argv) :
  file_(std::string(argv[1])) {
}

void Config::Open() {
  this->Open(file_);
}

void Config::Open(std::string file) {
  Util::AssertFileExists(file);
  fs_.open(file, cv::FileStorage::READ);

  M2dp::theta_bins       = fs_["descriptor"]["m2dp"]["theta_bins"];
  M2dp::rho_bins         = fs_["descriptor"]["m2dp"]["rho_bins"];
  M2dp::azimuth_angles   = fs_["descriptor"]["m2dp"]["azimuth_angles"];
  M2dp::elevation_angles = fs_["descriptor"]["m2dp"]["elevation_angles"];

  CM2dp::theta_bins       = fs_["descriptor"]["cm2dp"]["theta_bins"];
  CM2dp::rho_bins         = fs_["descriptor"]["cm2dp"]["rho_bins"];
  CM2dp::azimuth_angles   = fs_["descriptor"]["cm2dp"]["azimuth_angles"];
  CM2dp::elevation_angles = fs_["descriptor"]["cm2dp"]["elevation_angles"];
  CM2dp::color_bins       = fs_["descriptor"]["cm2dp"]["color_bins"];
  CM2dp::color_scheme     = fs_["descriptor"]["cm2dp"]["color_scheme"];

  BruteforceMatcher::k         = fs_["matcher"]["bruteforce"]["k"];
  BruteforceMatcher::window    = fs_["matcher"]["bruteforce"]["window"];
  BruteforceMatcher::threshold = fs_["matcher"]["bruteforce"]["threshold"];

  Flann::k                = fs_["matcher"]["flann"]["k"];
  Flann::trees            = fs_["matcher"]["flann"]["trees"];
  Flann::recursive_checks = fs_["matcher"]["flann"]["recursive_checks"];
  Flann::window           = fs_["matcher"]["flann"]["window"];
  Flann::threshold        = fs_["matcher"]["flann"]["threshold"];

  NormalEstimation::scalar_radius = fs_["normal"]["least_squares"]["scalar_radius"];

}

void Config::Load(Pipeline &pipeline) {
  std::string file, extension, query, database, descriptor, normal, matcher;

  pipeline.task(fs_["general"]["task"]);

  fs_["distance_matrix"]["file"]["name"] >> file;
  fs_["distance_matrix"]["file"]["ext"] >> extension;
  fs_["general"]["descriptor"] >> descriptor;
  fs_["general"]["normal"] >> normal;
  fs_["general"]["matcher"] >> matcher;
  fs_["dataset"]["query"] >> query;
  fs_["dataset"]["database"] >> database;

  file.append(descriptor);
  file.append(".");
  file.append(extension);
  pipeline.output_.distance_matrix(file);

  this->Load(pipeline.query(), query);
  this->Load(pipeline.database(), database);

  pcl::search::KdTree<PointType>::Ptr kdtree = boost::make_shared<pcl::search::KdTree<PointType> >();

  if (descriptor == "shot")
    pipeline.descriptor(boost::make_shared<ShotEstimation<Shot> >(kdtree));
  else if (descriptor == "cshot")
    pipeline.descriptor(boost::make_shared<ShotEstimation<CShot> >(kdtree));
  else if (descriptor == "m2dp")
    pipeline.descriptor(boost::make_shared<M2dpEstimation<M2dp> >());
  else if (descriptor == "cm2dp")
    pipeline.descriptor(boost::make_shared<M2dpEstimation<CM2dp> >());

  if (descriptor == "shot" || descriptor == "cshot")
    if (normal == "ls")
      pipeline.descriptor()->normal(boost::make_shared<NormalEstimation>(kdtree));

  if (matcher == "bruteforce")
    pipeline.matcher(boost::make_shared<BruteforceMatcher>());
  //else if (matcher == "flann")
}

void Config::Load(Dataset &dataset, std::string sequence) {
  std::string descriptor;
  fs_["general"]["descriptor"] >> descriptor;

  dataset.output(sequence+"_"+descriptor);

  dataset.path      (fs_["dataset"][sequence]["path"]);
  dataset.amount    (fs_["dataset"][sequence]["amount"]);
  dataset.frames    (fs_["dataset"][sequence]["frames"]);
  dataset.poses     (fs_["dataset"][sequence]["poses"]);
  dataset.format    (fs_["dataset"][sequence]["format"]);
  dataset.distance  (fs_["dataset"][sequence]["distance"]);
  dataset.timestamps(fs_["dataset"][sequence]["timestamps"]);
  dataset.mask      (fs_["dataset"][sequence]["mask"]);
  dataset.downsample(fs_["dataset"]["downsample"]);
}

void Config::Close() {
  fs_.release();
}

} // namespace prototype2
