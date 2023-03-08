#include "pipeline.h"

namespace prototype2 {

Pipeline::Pipeline(int argc, char **argv) : QObject() {
  current_ = -1;
  //thread_distances_ = new QThread();
  //this->connect(this, SIGNAL(ShowMatrix(int, int)), this, SLOT(MatrixOutput(int, int)));
  //this->connect(this, SIGNAL(ShowCurrentTrajectory(int)), &trajectory_widget_, SLOT(UpdateTrajectory(int)));
}

void Pipeline::Run() {
  //QGuiApplication::setApplicationDisplayName(DistancesWidget::tr("Viewer"));
  //QGuiApplication::setApplicationDisplayName(TrajectoryWidget::tr("Viewer"));
  //distances_widget_.resize(800, 600);
  //trajectory_widget_.resize(800, 600);
  if (task_ == Task::BUILD) {
    this->Build();
  } else if (task_ == Task::QUERY) {
    //trajectory_widget_.show();
	this->Query();
    //trajectory_widget_.update();
  } else if (task_ == Task::MATRIX) {
    //distances_widget_.show();
    this->Matrix();
    //distances_widget_.update();
  } else if (task_ == Task::PR) {
	this->PrecisionRecall();
  } else if (task_ == Task::MATLAB) {
    this->Matlab();
  }
  task_ = Task::END;
  /*
  database_.Load();
  boost::shared_ptr<Eigen::Array<double, Eigen::Dynamic, Eigen::Dynamic> > translation =
    boost::make_shared<Eigen::Array<double, Eigen::Dynamic, Eigen::Dynamic> >(database_.size(), 3);
  database_.Load(translation);
  trajectory_widget_.trajectory(translation);
  trajectory_widget_.show();
  std::cout << "testewwww" << std::endl;


  /*if (task_ == Task::BUILD) {
    this->Build();
    task_ = Task::END;
  } else if (task_ == Task::QUERY) {

  } else if (task_ == Task::MATRIX) {
    //QTimer timer(this);
    window_.show();
    this->Matrix();
    window_.update();
  }

    //QObject::connect(thread_distances_, SIGNAL(started()), this, SLOT(Matrix()));
    //QObject::connect(this, SIGNAL(ShowMatrix()), &window_, SLOT(update()));
    //std::cout << "teste123" << std::endl;
    //thread_distances_->start();
    //std::cout << "teste456" << std::endl;
    //while (task_ != Task::END) {
    //  sleep(250);

    //}
    //std::cout << "tessste" << std::endl;
    //thread_distances_.wait();
    //timer.start(1000);
    //boost::thread thread(&Pipeline::Matrix, this);
    //window_.distances(distances_);
    /*int current = -1;
    while (task_ != Task::END) {
      if (current != current_) {
        current = current_;
        window_.UpdateDistanceMatrix(distances_);
      }
    }*/
    //thread.join();
  //}


  /*if (task_ == Task::BUILD) {
    this->Build();
    task_ = Task::END;
  } else if (task_ == Task::QUERY) {
    boost::thread thread(&Pipeline::QueryOutput, this);
    this->Query();
    task_ = Task::END;
    thread.join();
  } else if (task_ == Task::MATRIX) {
    //boost::thread thread(&Pipeline::DistanceMatrixOutput, this);
    this->Matrix();
    task_ = Task::END;
    //thread.join();
  }*/
}

void Pipeline::Build() {
  boost::shared_ptr<cv::Mat> descriptors = boost::make_shared<cv::Mat>();
  Eigen::Array<double, 1, Eigen::Dynamic> times;
  times.resize(database_.amount());
  for (int i = 0; i < database_.amount(); i++) {
    std::cout << i << " -----------------------" << std::endl;
    pcl::PointCloud<PointType>::Ptr cloud = database_.Frame(i);
    double time_start = pcl::getTime();
    descriptor_->Extract(cloud);
    times(i) = double(pcl::getTime() - time_start);
    std::cout << database_.output() << " took " << times(i) << std::endl;
    descriptors->push_back(descriptor_->result());
  }
  database_.Save(descriptors);

  std::ofstream file;
  file.open("build_"+database_.output()+".csv");
  file << times;
  file.close();
}

void Pipeline::Matlab() {
  boost::shared_ptr<cv::Mat> descriptors = boost::make_shared<cv::Mat>(1101, 192, CV_32F);

  std::string line;
  std::ifstream input;
  input.open("kitti06.csv");  
  for (int i = 0; i < 1101; i++) {
    for (int j = 0; j < 192; j++) { 
      std::string value;
      std::getline(input, value, ';'); 
      std::cout << value << std::endl;
   	  descriptors->at<float>(i, j) = (float)std::stod(value);
    }
    std::getline(input, line, '\n');     
  }
  input.close();
  database_.Save(descriptors);
}

void Pipeline::Query() {
  boost::shared_ptr<cv::Mat> descriptors = boost::make_shared<cv::Mat>();
  boost::shared_ptr<Eigen::Array<double, Eigen::Dynamic, Eigen::Dynamic> > poses
    = boost::make_shared<Eigen::Array<double, Eigen::Dynamic, Eigen::Dynamic> >();
  int sequence_length;
  //trajectory_widget_.trajectory(poses);
  database_.Load(descriptors, poses);
  matcher_->Train(descriptors);
  sequence_length = descriptors->rows;
  for (int i = 0; i < sequence_length; i++) {
    current_ = i;
    //step1_.descriptor()->Extract(query_.Frame(i));
    matcher_->Window(i);
    //step1_.matcher()->KnnMatch(step1_.descriptor()->result());
    matcher_->KnnMatch(descriptors->row(i));
    int result = matcher_->Result();
    double distance = -1;
    if (result != -1) distance = Util::L2norm(poses->row(i), poses->row(result));
    std::cout << "c " << i << " r " << result << " d " << distance << std::endl;
    std::cout << "---------" << std::endl;
    emit ShowCurrentTrajectory(i);
  }
}

void Pipeline::PrecisionRecall() {
  boost::shared_ptr<cv::Mat> descriptors = boost::make_shared<cv::Mat>();
  boost::shared_ptr<Eigen::Array<double, Eigen::Dynamic, Eigen::Dynamic> > poses
    = boost::make_shared<Eigen::Array<double, Eigen::Dynamic, Eigen::Dynamic> >();
  std::vector<double> matches;  
  std::vector<double> thresholds;  
  std::vector<double> precision;  
  std::vector<double> recall;  
  std::vector<int> results;
  int sequence_length;

  BruteforceMatcher::threshold = 0;
  
  database_.Load(descriptors, poses);
  sequence_length = descriptors->rows;
  results.resize(sequence_length);
  matches.resize(sequence_length);

  matcher_->Train(descriptors);

  Eigen::Array<double, 1, Eigen::Dynamic> times;
  times.resize(sequence_length);
  for (int i = 0; i < sequence_length; i++) {
    current_ = i;
    matcher_->Window(i);
    double time_start = pcl::getTime();
    matcher_->KnnMatch(descriptors->row(i));
    times(i) = double(pcl::getTime() - time_start);
    std::cout << database_.output() << " matching took " << times(i) << std::endl;
    results[i] = matcher_->Result();
    if (results[i] != -1) {
      matches[i] = matcher_->matches()[results[i]][0][0].distance;
      thresholds.push_back(matches[i]);
    } else {
      matches[i] = std::numeric_limits<float>::max();
    }
  }

  std::ofstream file_times;
  file_times.open("matching_"+database_.output()+".csv");
  file_times << times;
  file_times.close();

  std::sort(thresholds.begin(), thresholds.end());
  double previous = -2;
  for (int i = 0; i < thresholds.size(); i++) { //thresholds
    if (thresholds[i] != previous) {
      int TP = 0, FP = 0, FN = 0, TN = 0;
      for (int j = 0; j < matches.size(); j++) { //matches
		    if (thresholds[i] >= matches[j]) { //loop found
          if (Util::L2norm(poses->row(j), poses->row(results[j])) < 10) TP++;//lower than 10m, nailed it
          else FP++;//failed
        } else { //loop not found
          if (results[j] == -1) TN++;//no loop, found nothing, nailed it
          else if (Util::L2norm(poses->row(j), poses->row(results[j])) < 10) FN++;//lower than 10m, there was a loop
          else TN++;//no loop, found nothing, nailed it
        }
      }
      //std::cout << "TP " << TP << std::endl;
      //std::cout << "FP " << FP << std::endl;
      //std::cout << "FN " << FN << std::endl;
      //std::cout << "TN " << TN << std::endl;
      double p = (double)TP/(TP+FP);
      double r = (double)TP/(TP+FN);
      //std::cout << "precision " << p << std::endl;
      //std::cout << "recall " << r << std::endl;
      precision.push_back(p);
      recall.push_back(r);
    }
    if (thresholds[i] != -1) previous = thresholds[i];
  }
  
  std::ofstream file_pr;
  file_pr.open("precisionrecall_"+database_.output()+".csv");
  file_pr << "x;y" << std::endl;
  for (int i = 0; i < precision.size(); i++) {
    file_pr << recall[i] << ";" << precision[i] << std::endl;
  }
  file_pr.close();
}

void Pipeline::Matrix() {
  boost::shared_ptr<Eigen::Array<double, Eigen::Dynamic, Eigen::Dynamic> > distances
    = boost::make_shared<Eigen::Array<double, Eigen::Dynamic, Eigen::Dynamic> >();
  boost::shared_ptr<Eigen::Array<double, Eigen::Dynamic, Eigen::Dynamic> > poses
    = boost::make_shared<Eigen::Array<double, Eigen::Dynamic, Eigen::Dynamic> >();
  boost::shared_ptr<cv::Mat> descriptors = boost::make_shared<cv::Mat>();

  //distances_widget_.distances(distances);
  //this->moveToThread(thread_distances_);
  database_.Load(descriptors, poses);
  matcher_->Train(descriptors);
  distances->resize(descriptors->rows, descriptors->rows);
  distances->setOnes();
  //boost::thread thread(&Pipeline::MatrixOutput, this);
  //cv::Mat distance_matrix(100, 100, CV_8UC1);
  cv::Mat distance_matrix = cv::Mat(descriptors->rows, descriptors->rows, CV_8UC1);
  //database_.descriptors().rows
  for (int i = 0; i < distances->rows(); i++) {
    matcher_->KnnMatch(descriptors->row(i));
    current_ = i;
    for (int j = 0; j < distances->cols(); j++) {
      //output_.distance_matrix().at<uchar>(i, j) =
      //  static_cast<int>(std::floor(step1_.matcher->matches()[j][0][0].distance * 255));
      //distance_matrix_.at<uchar>(i, j) =
      //  static_cast<int>(std::floor(step1_.matcher->BestMatchDistanceNormalized(j, i) * 255));
      //distances_[(i+1)*(j+1)] = step1_.matcher->matches()[j][0][0].distance;
      //distance_matrix.at<uchar>(i, j) =
      //  static_cast<int>(std::floor(step1_.matcher->BestMatchDistanceNormalized(j) * 255));
        //if (step1_.matcher->BestMatchDistanceNormalized(j) <= 0.4) {
          distance_matrix.at<uchar>(i, j) =
            static_cast<int>(std::floor(matcher_->BestMatchDistanceNormalized(j) * 255));
          //(*distances)(i, j) = matcher_->BestMatchDistanceNormalized(j);
        //} else {
        //  distance_matrix.at<uchar>(i, j) = 255;
        //  (*distances)(i, j) = 1;
        //}
     // emit HasNewMatrix(distances_);
    }
    //emit ShowMatrix();
  }
  task_ = Task::END;
  cv::imshow("Viewer", distance_matrix);
  cv::imwrite("04_distances_kitti00_m2dp.jpg", distance_matrix);
  cv::waitKey(10);
  //thread.join();
  //output_.SaveDistanceMatrix(matches);
  //output_.ShowDistanceMatrix(distance_matrix);
}

void Pipeline::QueryOutput() {
  /*pcl::PointCloud<PointType>::ConstPtr query_cloud, result_cloud;
  int query, result, query_aux = -1;
  while (current_ < dataset_.size()-1) {
    if (step1_.result > -2) {
      query = current_;
      if (query != query_aux) {
        result       = step1_.result;
        query_cloud  = dataset_.QueryFrame(query);
        if (result != -1) result_cloud = dataset_.Frame(result);
        else result_cloud = 0;
        query_aux    = query;
      }
      output_.ShowQueryResult(query_cloud, result_cloud);
    }
  }*/
}

void Pipeline::task(int code) {
  if (code == 1)      task_ = Task::BUILD;
  else if (code == 2) task_ = Task::QUERY;
  else if (code == 3) task_ = Task::MATRIX;
  else if (code == 4) task_ = Task::PR;
  else if (code == 5) task_ = Task::MATLAB;
  else                task_ = Task::QUERY;
}

Dataset& Pipeline::query() {
  return query_;
}

Dataset& Pipeline::database() {
  return database_;
}

void Pipeline::descriptor(Descriptor::Ptr descriptor) {
  descriptor_ = descriptor;
}

void Pipeline::matcher(Matcher::Ptr matcher) {
  matcher_ = matcher;
}

Descriptor::Ptr Pipeline::descriptor() {
  return descriptor_;
}

Matcher::Ptr Pipeline::matcher() {
  return matcher_;
}

} // namespace prototype2
