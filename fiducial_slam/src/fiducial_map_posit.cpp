//
// Created by ejalaa on 20/07/2021.
//

#include <fiducial_slam/helpers.h>
#include <ros/ros.h>
#include <fiducial_msgs/FiducialTransform.h>
#include <fiducial_msgs/FiducialTransformArray.h>
#include <fiducial_slam/map.h>
#include <csignal>

using namespace std;
using namespace cv;

// ==========================================================================================
// MOVING MAP
// ==========================================================================================
class MovingMap
{
public:
  tf2_ros::TransformBroadcaster broadcaster;
  tf2_ros::Buffer tfBuffer;
  std::unique_ptr<tf2_ros::TransformListener> listener;

  ros::Publisher markerPub;
  ros::Publisher mapPub;

  std::string mapFilename;
  std::string mapFrame;
  std::string mMapFrame;
  std::string cameraFrame;
  std::string baseFrame;

  double future_date_transforms;
  bool havePose;
  float tfPublishInterval;
  bool publishPoseTf;
  ros::Time tfPublishTime;
  geometry_msgs::TransformStamped poseTf;

  double systematic_error;
  std::map<int, Fiducial> fiducials;

  std::vector<double> covarianceDiagonal;
  bool loadMap();
  bool loadMap(string filename);

  bool lookupTransform(const std::string &from, const std::string &to, const ros::Time &time,
                       tf2::Transform &T) const;
  MovingMap(ros::NodeHandle &nh);
  void publishMarkers();
  void publishMarker(Fiducial &fid);

  void publishTf();
  void updateMapPose(std::vector<Observation> &obs, const ros::Time &time);
  void update();
};

MovingMap::MovingMap(ros::NodeHandle &nh) : tfBuffer(ros::Duration(30))
{
  havePose = false;
  listener = make_unique<tf2_ros::TransformListener>(tfBuffer);
  markerPub = ros::Publisher(nh.advertise<visualization_msgs::Marker>("/fiducials", 100));
  mapPub = ros::Publisher(nh.advertise<fiducial_msgs::FiducialMapEntryArray>("/fiducial_map", 1));

  nh.param<std::string>("map_frame", mapFrame, "map");
  nh.param<std::string>("mmap_frame", mMapFrame, "mmap");
  nh.param<std::string>("base_frame", baseFrame, "base_footprint");
  nh.param<float>("tf_publish_interval", tfPublishInterval, 1.0);
  nh.param<double>("future_date_transforms", future_date_transforms, 0.1);
  nh.param<bool>("publish_tf", publishPoseTf, true);
  nh.param<double>("systematic_error", systematic_error, 0.01);
  std::fill(covarianceDiagonal.begin(), covarianceDiagonal.end(), 0);
  nh.param<std::string>("map_file", mapFilename,
                        std::string(getenv("HOME")) + "/.ros/slam/map.txt");

  loadMap();
  publishMarkers();

}

bool MovingMap::loadMap()
{
  return loadMap(mapFilename);
}
bool MovingMap::loadMap(string filename)
{
  int numRead = 0;

  ROS_INFO("Load map %s", filename.c_str());

  FILE * fp = fopen(filename.c_str(), "r");
  if (fp == NULL) {
    ROS_WARN("Could not open %s for read\n", filename.c_str());
    return false;
  }

  const int BUFSIZE = 2048;
  char linebuf[BUFSIZE];
  char linkbuf[BUFSIZE];

  while (!feof(fp)) {
    if (fgets(linebuf, BUFSIZE - 1, fp) == NULL) { break; }

    int id;
    double tx, ty, tz, rx, ry, rz, var;
    int numObs = 0;

    linkbuf[0] = '\0';
    int nElems = sscanf(linebuf, "%d %lf %lf %lf %lf %lf %lf %lf %d%[^\t\n]*s", &id, &tx, &ty,
                        &tz, &rx, &ry, &rz, &var, &numObs, linkbuf);
    if (nElems == 9 || nElems == 10) {
      tf2::Vector3 tvec(tx, ty, tz);
      tf2::Quaternion q;
      q.setRPY(deg2rad(rx), deg2rad(ry), deg2rad(rz));

      auto twv = TransformWithVariance(tvec, q, var);
      // TODO: figure out what the timestamp in Fiducial should be
      Fiducial f =
          Fiducial(id, tf2::Stamped<TransformWithVariance>(twv, ros::Time::now(), mapFrame));
      f.numObs = numObs;

      std::istringstream ss(linkbuf);
      std::string s;
      while (getline(ss, s, ' ')) {
        if (!s.empty()) {
          f.links.insert(stoi(s));
        }
      }
      fiducials[id] = f;
      numRead++;
    } else {
      ROS_WARN("Invalid line: %s", linebuf);
    }
  }

  fclose(fp);
  ROS_INFO("Load map %s read %d entries", filename.c_str(), numRead);
  return true;
}
bool MovingMap::lookupTransform(const string &from,
                                const string &to,
                                const ros::Time &time,
                                tf2::Transform &T) const
{
  geometry_msgs::TransformStamped transform;

  try {
    transform = tfBuffer.lookupTransform(from, to, time, ros::Duration(0.5));

    tf2::fromMsg(transform.transform, T);
    return true;
  } catch (tf2::TransformException &ex) {
    ROS_WARN("%s", ex.what());
    return false;
  }
}
void MovingMap::publishMarkers()
{
  ros::Time now = ros::Time::now();
  std::map<int, Fiducial>::iterator it;

  for (auto &map_pair : fiducials) {
    Fiducial &f = map_pair.second;
    if ((now - f.lastPublished).toSec() > 1.0) {
      publishMarker(f);
    }
  }
}

void MovingMap::publishMarker(Fiducial &fid)
{
  fid.lastPublished = ros::Time::now();

  // Flattened cube
  visualization_msgs::Marker marker;
  marker.type = visualization_msgs::Marker::CUBE;
  marker.action = visualization_msgs::Marker::ADD;
  toMsg(fid.pose.transform, marker.pose);

  marker.scale.x = 0.15;
  marker.scale.y = 0.15;
  marker.scale.z = 0.01;
  if (fid.visible) {
    marker.color.r = 1.0f;
    marker.color.g = 0.0f;
    marker.color.b = 0.0f;
    marker.color.a = 1.0f;
  } else {
    marker.color.r = 0.0f;
    marker.color.g = 1.0f;
    marker.color.b = 0.0f;
    marker.color.a = 1.0f;
  }
  marker.id = fid.id;
  marker.ns = "fiducial";
  marker.header.frame_id = mMapFrame;
  markerPub.publish(marker);

  // cylinder scaled by stddev
  visualization_msgs::Marker cylinder;
  cylinder.type = visualization_msgs::Marker::CYLINDER;
  cylinder.action = visualization_msgs::Marker::ADD;
  cylinder.header.frame_id = mMapFrame;
  cylinder.color.r = 0.0f;
  cylinder.color.g = 0.0f;
  cylinder.color.b = 1.0f;
  cylinder.color.a = 0.5f;
  cylinder.id = fid.id + 10000;
  cylinder.ns = "sigma";
  cylinder.scale.x = cylinder.scale.y = std::max(std::sqrt(fid.pose.variance), 0.1);
  cylinder.scale.z = 0.01;
  cylinder.pose.position.x = marker.pose.position.x;
  cylinder.pose.position.y = marker.pose.position.y;
  cylinder.pose.position.z = marker.pose.position.z;
  cylinder.pose.position.z += (marker.scale.z / 2.0) + 0.05;
  markerPub.publish(cylinder);

  // Text
  visualization_msgs::Marker text;
  text.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
  text.action = visualization_msgs::Marker::ADD;
  text.header.frame_id = mMapFrame;
  text.color.r = text.color.g = text.color.b = text.color.a = 1.0f;
  text.id = fid.id;
  text.scale.x = text.scale.y = text.scale.z = 0.1;
  text.pose.position.x = marker.pose.position.x;
  text.pose.position.y = marker.pose.position.y;
  text.pose.position.z = marker.pose.position.z;
  text.pose.position.z += (marker.scale.z / 2.0) + 0.1;
  text.id = fid.id + 30000;
  text.ns = "text";
  text.text = std::to_string(fid.id);
  markerPub.publish(text);

  // Links
  visualization_msgs::Marker links;
  links.type = visualization_msgs::Marker::LINE_LIST;
  links.action = visualization_msgs::Marker::ADD;
  links.header.frame_id = mMapFrame;
  links.color.r = 0.0f;
  links.color.g = 0.0f;
  links.color.b = 1.0f;
  links.color.a = 1.0f;
  links.id = fid.id + 40000;
  links.ns = "links";
  links.scale.x = links.scale.y = links.scale.z = 0.02;
  links.pose.position.x = 0;
  links.pose.position.y = 0;
  links.pose.position.z = 0;

  geometry_msgs::Point gp0, gp1;
  tf2::Vector3 p0 = fid.pose.transform.getOrigin();
  gp0.x = p0.x();
  gp0.y = p0.y();
  gp0.z = p0.z();

  std::map<int, int>::iterator lit;
  for (const auto linked_fid : fid.links) {
    // only draw links in one direction
    if (fid.id < linked_fid) {
      if (fiducials.find(linked_fid) != fiducials.end()) {
        tf2::Vector3 p1 = fiducials[linked_fid].pose.transform.getOrigin();
        gp1.x = p1.x();
        gp1.y = p1.y();
        gp1.z = p1.z();
        links.points.push_back(gp0);
        links.points.push_back(gp1);
      }
    }
  }

  markerPub.publish(links);
}

void MovingMap::updateMapPose(vector<Observation> &obs, const ros::Time &time)
{
  ROS_INFO("Updating map with %d observations. Map has %d fiducials", (int) obs.size(),
           (int) fiducials.size());

  tf2::Stamped<TransformWithVariance> T_res;
  T_res.frame_id_ = mapFrame;

  int numEsts = 0;
  tf2::Stamped<TransformWithVariance> T_camBase;
  tf2::Stamped<TransformWithVariance> T_baseCam;
  tf2::Stamped<TransformWithVariance> T_fidmapBase;
  tf2::Stamped<TransformWithVariance> T_baseMap;

  if (obs.size() == 0) {
    return;
  }

  if (lookupTransform(obs[0].T_camFid.frame_id_, baseFrame, time, T_camBase.transform)) {
    tf2::Vector3 c = T_camBase.transform.getOrigin();
    ROS_INFO("camera->base   %lf %lf %lf", c.x(), c.y(), c.z());
    T_camBase.variance = 1.0;
  } else {
    ROS_ERROR("Cannot determine tf from camera to robot\n");
  }

  if (lookupTransform(baseFrame, obs[0].T_camFid.frame_id_, time, T_baseCam.transform)) {
    tf2::Vector3 c = T_baseCam.transform.getOrigin();
    ROS_INFO("base->camera   %lf %lf %lf", c.x(), c.y(), c.z());
    T_baseCam.variance = 1.0;
  } else {
    ROS_ERROR("Cannot determine tf from robot to camera\n");
    return;
  }

  if (lookupTransform(baseFrame, mapFrame, time, T_baseMap.transform)) {
    tf2::Vector3 c = T_baseMap.transform.getOrigin();
    ROS_INFO("map->base   %lf %lf %lf", c.x(), c.y(), c.z());
    T_baseCam.variance = 1.0;
  } else {
    ROS_ERROR("Cannot determine tf from map to base\n");
    return;
  }

  for (Observation &o : obs) {
    if (fiducials.find(o.fid) != fiducials.end()) {
      const Fiducial &fid = fiducials[o.fid];

      // fidmapCam = fidmapFid * FidCam
      tf2::Stamped<TransformWithVariance> p = fid.pose * o.T_fidCam;

      p.frame_id_ = mMapFrame;
      p.stamp_ = o.T_fidCam.stamp_;

      // fidmapBase = fidmapCam * camBase
      p.setData(p * T_camBase);

      if (numEsts == 0) T_res = p;
      else {
        T_res.setData(averageTransforms(T_res, p));
        T_res.stamp_ = p.stamp_;
      }
      numEsts++;
    }
  }

  if (numEsts == 0) {
    ROS_INFO("Finished frame - no estimates\n");
    return;
  }

  T_res = T_res * T_baseMap;

  poseTf.transform = toMsg(T_res.transform.inverse());
  poseTf.header.frame_id = mapFrame;
  poseTf.child_frame_id = mMapFrame;

  //poseTf = toMsg(T_res);
  //poseTf.child_frame_id = "fid_map";

  havePose = true;
  if (publishPoseTf) publishTf();

}

void MovingMap::update()
{
  ros::Time now = ros::Time::now();
  if (publishPoseTf && havePose && tfPublishInterval != 0.0 &&
      (now - tfPublishTime).toSec() > tfPublishInterval) {
    publishTf();
    tfPublishTime = now;
  }
  publishMarkers();

}

void MovingMap::publishTf()
{
  tfPublishTime = ros::Time::now();
  poseTf.header.stamp = tfPublishTime + ros::Duration(future_date_transforms);
  broadcaster.sendTransform(poseTf);
}

// ==========================================================================================
// FIDUCIAL POSIT
// ==========================================================================================

class FiducialMapPosit
{
private:
  ros::Subscriber ft_sub;
  bool use_fiducial_area_as_weight;
  double weighting_scale;
  void transformCallback(const fiducial_msgs::FiducialTransformArray::ConstPtr &msg);

public:
  MovingMap fiducialMap;
  FiducialMapPosit(ros::NodeHandle &nh);
};

FiducialMapPosit::FiducialMapPosit(ros::NodeHandle &nh) : fiducialMap(nh)
{

  // If set, use the fiducial area in pixels^2 as an indication of the
  // 'goodness' of it. This will favor fiducials that are close to the
  // camera and center of the image. The reciprical of the area is actually
  // used, in place of reprojection error as the estimate's variance
  nh.param<bool>("use_fiducial_area_as_weight", use_fiducial_area_as_weight, false);
  // Scaling factor for weighing
  nh.param<double>("weighting_scale", weighting_scale, 1e9);

  ft_sub = nh.subscribe("/fiducial_transforms", 1, &FiducialMapPosit::transformCallback, this);

  ROS_INFO("Fiducial Slam ready");

}
void FiducialMapPosit::transformCallback(const fiducial_msgs::FiducialTransformArray::ConstPtr &msg)
{
  vector<Observation> observations;

  for (size_t i = 0; i < msg->transforms.size(); i++) {
    const fiducial_msgs::FiducialTransform &ft = msg->transforms[i];

    tf2::Vector3 tvec(ft.transform.translation.x, ft.transform.translation.y,
                      ft.transform.translation.z);

    tf2::Quaternion q(ft.transform.rotation.x, ft.transform.rotation.y, ft.transform.rotation.z,
                      ft.transform.rotation.w);

    double variance;
    if (use_fiducial_area_as_weight) {
      variance = weighting_scale / ft.fiducial_area;
    } else {
      variance = weighting_scale * ft.object_error;
    }

    Observation obs(ft.fiducial_id, tf2::Stamped<TransformWithVariance>(
        TransformWithVariance(ft.transform, variance),
        msg->header.stamp, msg->header.frame_id));
    observations.push_back(obs);
  }
  fiducialMap.updateMapPose(observations, msg->header.stamp);
}



// ================================================================================================
// Main
// ================================================================================================

auto node = unique_ptr<FiducialMapPosit>(nullptr);
void mySigintHandler(int sig)
{

  ros::shutdown();
}
int main(int argc, char ** argv)
{
  ros::init(argc, argv, "fiducial_map_posit", ros::init_options::NoSigintHandler);
  ros::NodeHandle nh("~");

  node = make_unique<FiducialMapPosit>(nh);
  signal(SIGINT, mySigintHandler);

  ros::Rate r(20);

  while (ros::ok()) {
    ros::spinOnce();
    r.sleep();
    node->fiducialMap.update();
  }

  return 0;
}