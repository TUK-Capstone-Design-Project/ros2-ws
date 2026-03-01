#include "ConfigSettings/ConfigSettings.hpp"
#include <algorithm> // for std::clamp
#include <filesystem>
#include <fstream>
#include <iostream>


namespace LCODE {

auto ConfigSettings::endsWith(const std::string &str, const std::string &suffix) -> bool {
  return str.size() >= suffix.size() && str.compare(str.size() - suffix.size(), suffix.size(), suffix) == 0;
}

auto ConfigSettings::createConfigurationFile(std::string filePath) -> bool {
  nlohmann::json config_json;
  std::ofstream  file(std::filesystem::u8path((const char *)(filePath).c_str()));
  if (!file.is_open()) {
    std::cerr << "파일을 열 수 없습니다: " << filePath << std::endl;
    return false;
  }
  this->LBS_Config_Path = filePath;

  // System
  config_json["System"]["LBS_Config_Path"] = "./";

  // ROI
  config_json["ROI"]["ROI_width_dot"]  = 13;
  config_json["ROI"]["ROI_height_dot"] = 13;

  // Debruijn_Params
  config_json["Debruijn_Params"]["alpha"]  = "0123";
  config_json["Debruijn_Params"]["subRow"] = 5;
  config_json["Debruijn_Params"]["subCol"] = 5;

  // Detection_Params
  config_json["Detection_Params"]["maxAngle"]                 = 17.0;
  config_json["Detection_Params"]["EPS"]                      = 5.5;
  config_json["Detection_Params"]["line_size"]                = 900;
  config_json["Detection_Params"]["clusterDistanceTolerance"] = 40;

  // Blob_Detector -> Threshold
  config_json["Blob_Detector"]["Threshold"]["min"]                 = 60;
  config_json["Blob_Detector"]["Threshold"]["max"]                 = 220;
  config_json["Blob_Detector"]["Threshold"]["step"]                = 15;
  config_json["Blob_Detector"]["Threshold"]["minRepeatability"]    = 2;
  config_json["Blob_Detector"]["Threshold"]["minDistBetweenBlobs"] = 5;

  // Blob_Detector -> Filter
  config_json["Blob_Detector"]["Filter"]["Area"]["enabled"] = false;
  config_json["Blob_Detector"]["Filter"]["Area"]["min"]     = 20;
  config_json["Blob_Detector"]["Filter"]["Area"]["max"]     = 5000;

  config_json["Blob_Detector"]["Filter"]["Circularity"]["enabled"] = true;
  config_json["Blob_Detector"]["Filter"]["Circularity"]["min"]     = 0.8f;
  config_json["Blob_Detector"]["Filter"]["Circularity"]["max"]     = 1.0f;

  config_json["Blob_Detector"]["Filter"]["Inertia"]["enabled"] = false;
  config_json["Blob_Detector"]["Filter"]["Inertia"]["min"]     = 0.1f;
  config_json["Blob_Detector"]["Filter"]["Inertia"]["max"]     = 1.0f;

  config_json["Blob_Detector"]["Filter"]["Convexity"]["enabled"] = false;
  config_json["Blob_Detector"]["Filter"]["Convexity"]["min"]     = 0.8f;
  config_json["Blob_Detector"]["Filter"]["Convexity"]["max"]     = 1.0f;

  file << config_json.dump(4);
  file.close();
  return true;
}

auto ConfigSettings::readConfigurationFile(std::string path) -> bool {
  std::string filePath;
  if (endsWith(path, ".json")) {
    filePath = path;
  } else {
    filePath = path + "LBS_config.json";
  }

  std::ifstream inputFile(std::filesystem::u8path((const char *)(filePath).c_str()));

  if (!inputFile.is_open()) {
#ifndef _RELEASE_MODE
    std::cout << "open failed!" << std::endl;
#endif //_RELEASE_MODE
    if (!createConfigurationFile(filePath)) {
      return false;
    }
    inputFile.open(std::filesystem::u8path((const char *)(filePath).c_str()));
  }

  nlohmann::json fs;
  inputFile >> fs;
  inputFile.close();

  // 안전하게 계층 구조를 읽어오기 위한 서브 JSON 객체 생성
  auto jSystem    = fs.value("System", nlohmann::json::object());
  auto jROI       = fs.value("ROI", nlohmann::json::object());
  auto jDebruijn  = fs.value("Debruijn_Params", nlohmann::json::object());
  auto jDetection = fs.value("Detection_Params", nlohmann::json::object());

  auto jBlob      = fs.value("Blob_Detector", nlohmann::json::object());
  auto jThreshold = jBlob.value("Threshold", nlohmann::json::object());
  auto jFilter    = jBlob.value("Filter", nlohmann::json::object());

  auto jArea        = jFilter.value("Area", nlohmann::json::object());
  auto jCircularity = jFilter.value("Circularity", nlohmann::json::object());
  auto jInertia     = jFilter.value("Inertia", nlohmann::json::object());
  auto jConvexity   = jFilter.value("Convexity", nlohmann::json::object());

  // 데이터 바인딩
  LBS_Config_Path = jSystem.value("LBS_Config_Path", "./");

  ROI_width_dot  = jROI.value("ROI_width_dot", 13);
  ROI_height_dot = jROI.value("ROI_height_dot", 13);

  alpha  = jDebruijn.value("alpha", "0123");
  subRow = jDebruijn.value("subRow", 5);
  subCol = jDebruijn.value("subCol", 5);

  maxAngle                 = jDetection.value("maxAngle", 17.0);
  EPS                      = jDetection.value("EPS", 5.5);
  line_size                = jDetection.value("line_size", 900);
  clusterDistanceTolerance = jDetection.value("clusterDistanceTolerance", 40);

  minThreshold        = jThreshold.value("min", 60);
  maxThreshold        = jThreshold.value("max", 220);
  thresholdStep       = jThreshold.value("step", 15);
  min_repeatability   = jThreshold.value("minRepeatability", 2);
  minDistBetweenBlobs = jThreshold.value("minDistBetweenBlobs", 5);

  filterByArea = jArea.value("enabled", false);
  minArea      = jArea.value("min", 20);
  maxArea      = jArea.value("max", 5000);

  // 1. Circularity (원형도)
  filterByCircularity = jCircularity.value("enabled", true);
  min_circularity     = std::clamp(jCircularity.value("min", 0.8f), 0.0f, 1.0f);
  max_circularity     = std::clamp(jCircularity.value("max", 1.0f), min_circularity, 1.0f);

  // 2. Inertia (관성비)
  filterByInertia = jInertia.value("enabled", false);
  min_inertia     = std::clamp(jInertia.value("min", 0.1f), 0.0f, 1.0f);
  max_inertia     = std::clamp(jInertia.value("max", 1.0f), min_inertia, 1.0f);

  // 3. Convexity (볼록성)
  filterByConvexity = jConvexity.value("enabled", false);
  min_convexity     = std::clamp(jConvexity.value("min", 0.8f), 0.0f, 1.0f);
  max_convexity     = std::clamp(jConvexity.value("max", 1.0f), min_convexity, 1.0f);

  return true;
}

ConfigSettings::ConfigSettings(std::string path) {
  this->loadSettings(path);
}

auto ConfigSettings::saveConfigSetting(std::string filePath) -> bool {
  if (filePath == "") {
    filePath = "./";
  } else if (filePath[filePath.size() - 1] != '/' && filePath[filePath.size() - 1] != '\\') {
    filePath.append(1, '/');
  }

  std::string    filename = filePath + "LBS_config.json";
  nlohmann::json config_json;
  std::ofstream  file(std::filesystem::u8path((const char *)(filename).c_str()));

  if (!file.is_open()) {
    std::cerr << "파일을 열 수 없습니다: " << filename << std::endl;
    return false;
  }

  // System
  config_json["System"]["LBS_Config_Path"] = this->LBS_Config_Path;

  // ROI
  config_json["ROI"]["ROI_width_dot"]  = this->ROI_width_dot;
  config_json["ROI"]["ROI_height_dot"] = this->ROI_height_dot;

  // Debruijn_Params
  config_json["Debruijn_Params"]["alpha"]  = this->alpha;
  config_json["Debruijn_Params"]["subRow"] = this->subRow;
  config_json["Debruijn_Params"]["subCol"] = this->subCol;

  // Detection_Params
  config_json["Detection_Params"]["maxAngle"]                 = this->maxAngle;
  config_json["Detection_Params"]["EPS"]                      = this->EPS;
  config_json["Detection_Params"]["line_size"]                = this->line_size;
  config_json["Detection_Params"]["clusterDistanceTolerance"] = this->clusterDistanceTolerance;

  // Blob_Detector -> Threshold
  config_json["Blob_Detector"]["Threshold"]["min"]                 = this->minThreshold;
  config_json["Blob_Detector"]["Threshold"]["max"]                 = this->maxThreshold;
  config_json["Blob_Detector"]["Threshold"]["step"]                = this->thresholdStep;
  config_json["Blob_Detector"]["Threshold"]["minRepeatability"]    = this->min_repeatability;
  config_json["Blob_Detector"]["Threshold"]["minDistBetweenBlobs"] = this->minDistBetweenBlobs;

  // Blob_Detector -> Filter
  config_json["Blob_Detector"]["Filter"]["Area"]["enabled"] = this->filterByArea;
  config_json["Blob_Detector"]["Filter"]["Area"]["min"]     = this->minArea;
  config_json["Blob_Detector"]["Filter"]["Area"]["max"]     = this->maxArea;

  config_json["Blob_Detector"]["Filter"]["Circularity"]["enabled"] = this->filterByCircularity;
  config_json["Blob_Detector"]["Filter"]["Circularity"]["min"]     = this->min_circularity;
  config_json["Blob_Detector"]["Filter"]["Circularity"]["max"]     = this->max_circularity;

  config_json["Blob_Detector"]["Filter"]["Inertia"]["enabled"] = this->filterByInertia;
  config_json["Blob_Detector"]["Filter"]["Inertia"]["min"]     = this->min_inertia;
  config_json["Blob_Detector"]["Filter"]["Inertia"]["max"]     = this->max_inertia;

  config_json["Blob_Detector"]["Filter"]["Convexity"]["enabled"] = this->filterByConvexity;
  config_json["Blob_Detector"]["Filter"]["Convexity"]["min"]     = this->min_convexity;
  config_json["Blob_Detector"]["Filter"]["Convexity"]["max"]     = this->max_convexity;

  // 보기 좋게 4칸 들여쓰기하여 저장
  file << config_json.dump(4);
  file.close();
  return true;
}

void ConfigSettings::loadSettings(std::string path) {
  if (path == "") {
    path = "./";
  }
  if (readConfigurationFile(path)) {
    loaded = true;
  }
}

auto ConfigSettings::getSubRow() const -> std::size_t {
  return this->subRow;
}
auto ConfigSettings::getSubCol() const -> std::size_t {
  return this->subCol;
}
auto ConfigSettings::getROI_width_dot() const -> std::size_t {
  return this->ROI_width_dot;
}
auto ConfigSettings::getROI_height_dot() const -> std::size_t {
  return this->ROI_height_dot;
}
auto ConfigSettings::getAlpha() const -> std::string {
  return this->alpha;
}
auto ConfigSettings::getMaxAngle() const -> float {
  return this->maxAngle;
}
auto ConfigSettings::getEPS() const -> float {
  return this->EPS;
}
auto ConfigSettings::setSubRow(int subRow_) -> void {
  this->subRow = subRow_;
}
auto ConfigSettings::setSubCol(int subCol_) -> void {
  this->subCol = subCol_;
}
auto ConfigSettings::setROI_width_dot(int ROI_width_dot_) -> void {
  this->ROI_width_dot = ROI_width_dot_;
}
auto ConfigSettings::setROI_height_dot(int ROI_height_dot_) -> void {
  this->ROI_height_dot = ROI_height_dot_;
}
auto ConfigSettings::setAlpha(std::string alpha_) -> void {
  this->alpha = alpha_;
}
auto ConfigSettings::setMaxAngle(float maxAngle_) -> void {
  this->maxAngle = maxAngle_;
}
auto ConfigSettings::setEPS(float EPS_) -> void {
  this->EPS = EPS_;
}

} // namespace LCODE