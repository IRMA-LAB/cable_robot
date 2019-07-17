/**
 * @file sensorsconfigjsonparser.cpp
 * @author Simone Comari
 * @date 17 Jul 2019
 * @brief This file includes definitions of class declared in sensorsconfigjsonparser.h.
 */

#include "utils/sensorsconfigjsonparser.h"

//--------- Public Functions ---------------------------------------------------------//

SensorsConfigJsonParser::SensorsConfigJsonParser() {}

bool SensorsConfigJsonParser::ParseFile(const std::string& filename,
                                        const bool verbose /* = false*/)
{
  std::cout << "Parsing file '" << filename << "'...\n";

  // Check file extension
  size_t found = filename.rfind(std::string(".json"));
  if (found == std::string::npos || found != filename.length() - 5)
  {
    std::cerr << "[ERROR] Invaild file type. Only '.json' files can be parsed."
              << std::endl;
    return false;
  }

  // Open file
  std::ifstream ifile(filename);
  if (!ifile.is_open())
  {
    std::cerr << "[ERROR] Could not open file " << filename << std::endl;
    return false;
  }

  // Parse JSON (generic) data
  json raw_data;
  ifile >> raw_data;
  ifile.close();

  // Extract information and arrange them properly
  file_parsed_ = ExtractConfig(raw_data);

  // Display data
  if (file_parsed_ && verbose)
    PrintConfig();

  return file_parsed_;
}

bool SensorsConfigJsonParser::ParseFile(const char* filename,
                                        const bool verbose /*= false*/)
{
  return ParseFile(std::string(filename), verbose);
}

bool SensorsConfigJsonParser::ParseFile(const QString& filename,
                                        const bool verbose /*= false*/)
{
  return ParseFile(filename.toStdString(), verbose);
}

bool SensorsConfigJsonParser::ParseFile(const std::string& filename,
                                        SensorsParams* params,
                                        const bool verbose /*= false*/)
{
  if (ParseFile(filename, verbose))
  {
    GetConfigStruct(params);
    return true;
  }
  return false;
}

bool SensorsConfigJsonParser::ParseFile(const char* filename, SensorsParams* params,
                                        const bool verbose /*= false*/)
{
  return ParseFile(std::string(filename), params, verbose);
}

bool SensorsConfigJsonParser::ParseFile(const QString& filename, SensorsParams* params,
                                        const bool verbose /*= false*/)
{
  return ParseFile(filename.toStdString(), params, verbose);
}

void SensorsConfigJsonParser::PrintConfig() const
{
  if (!file_parsed_)
  {
    std::cerr << "[ERROR] No file was parsed yet!" << std::endl;
    return;
  }

  std::cout << "VISION PARAMETERS\n============================="
            << "\n H_cam2world\t\t" << config_params_.vision.H_c2w
            << "\n H_board2platform\n"
            << config_params_.vision.H_b2p << " camera/camera_matrix\n"
            << config_params_.vision.camera.camera_matrix << " camera/camera_matrix\n"
            << config_params_.vision.camera.dist_coeff << " camera/dist_coeff\n"
            << std::endl;
}

//--------- Private Functions --------------------------------------------------------//

bool SensorsConfigJsonParser::ExtractConfig(const json& raw_data)
{
  if (!ExtractVisionConfig(raw_data))
    return false;
  return true;
}

bool SensorsConfigJsonParser::ExtractVisionConfig(const json& raw_data)
{
  if (raw_data.count("vision") != 1)
  {
    std::cerr << "[ERROR] Missing vision structure!" << std::endl;
    return false;
  }
  json vision = raw_data["vision"];
  std::string field;
  try
  {
    for (uint8_t i = 0; i < 3; i++)
    {
      field = "H_cam2world";
      config_params_.vision.H_c2w.SetRow(i + 1,
                                         vision[field].at(i).get<std::vector<double>>());
      field = "H_board2platform";
      config_params_.vision.H_b2p.SetRow(i + 1,
                                         vision[field].at(i).get<std::vector<double>>());
    }
  }
  catch (json::type_error)
  {
    std::cerr << "[ERROR] Missing or invalid vision parameter field: " << field
              << std::endl;
    return false;
  }

  return IsVisionConfigValid() &&
         CameraParamsJsonParser::decode(vision["camera"], config_params_.vision.camera);
}

bool SensorsConfigJsonParser::IsVisionConfigValid() const
{
  if (grabgeom::GetHomgTransfRot(config_params_.vision.H_c2w).IsPositiveDefinite() &&
      config_params_.vision.H_c2w.GetRow(4) == grabnum::MatrixXd<1, 4>({0, 0, 0, 1}))
  {
    std::cerr << "[ERROR] H_cam2world is not a valid homogeneous transformation matrix!"
              << std::endl;
    return false;
  }

  if (grabgeom::GetHomgTransfRot(config_params_.vision.H_b2p).IsPositiveDefinite() &&
      config_params_.vision.H_b2p.GetRow(4) == grabnum::MatrixXd<1, 4>({0, 0, 0, 1}))
  {
    std::cerr
      << "[ERROR] H_board2platform is not a valid homogeneous transformation matrix!"
      << std::endl;
    return false;
  }

  return true;
}
