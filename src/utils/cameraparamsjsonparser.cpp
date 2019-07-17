/**
 * @file cameraparamsjsonparser.cpp
 * @author Giovanni Melandri, Simone Comari
 * @date 17 Jul 2019
 * @brief Implementation of class declared in cameraparamsjsonparser.h.
 */

#include "utils/cameraparamsjsonparser.h"

CameraParamsJsonParser::CameraParamsJsonParser() {}

CameraParamsJsonParser::CameraParamsJsonParser(const std::string& io_filepath)
{
  decodeJson(io_filepath);
}

//--------- Public functions ---------------------------------------------------------//

CameraParams CameraParamsJsonParser::getCameraParams() const { return params_; }

void CameraParamsJsonParser::setCameraParams(const CameraParams& params_2_set)
{
  params_ = params_2_set;
}

bool CameraParamsJsonParser::writeJson(
  const std::string& o_filepath /*= SRCDIR "/output_calib.json"*/)
{
  return writeJson(params_, o_filepath);
}

bool CameraParamsJsonParser::writeJson(
  const CameraParams& params_in,
  const std::string& o_filepath /*= SRCDIR "/output_calib.json*/)
{
  // Adjust filename
  std::string str = o_filepath;
  size_t found    = str.find_last_of(".");
  if (found == std::string::npos)
    str.append(".json");
  else if (str.substr(found).compare(("json")) != 0)
    str.replace(found, std::string::npos, ".json");

  std::ofstream o_file(str);
  if (!o_file.is_open())
  {
    std::cerr << "[ERROR] Could not create file" << std::endl;
    return false;
  }

  // Extract and rearrange camera matrix in a json friendly format.
  std::vector<double> camera_matrix(params_in.camera_matrix.begin<double>(),
                                    params_in.camera_matrix.end<double>());
  // Safety checks.
  for (uchar i = 0; i < camera_matrix.size(); i++)
    if (camera_matrix[i] < 0)
    {
      std::cerr << "[ERROR] Invalid camera matrix, all elements must be "
                   "positive. Cannot write to file"
                << std::endl;
      return false;
    }
  if (camera_matrix[1] != 0.0 || camera_matrix[3] != 0.0 || camera_matrix[6] != 0.0 ||
      camera_matrix[7] != 0.0 || camera_matrix[8] != 1.0)
  {
    std::cerr << "[ERROR] Invalid camera matrix, wrong matrix "
                 "definition. Cannot write to file"
              << std::endl;
    return false;
  }

  // Extract and rearrange distortion coefficients in a json friendly format.
  std::vector<double> dist_coeff(params_in.dist_coeff.begin<double>(),
                                 params_in.dist_coeff.end<double>());

  // Actual write step.
  json parameters_2_write;
  parameters_2_write["camera_matrix"] = camera_matrix;
  parameters_2_write["dist_coeff"]    = dist_coeff;
  o_file << std::setw(2) << parameters_2_write << std::endl;

  o_file.close();
  return true;
}

CameraParams CameraParamsJsonParser::decodeJson(
  const std::string& i_filepath /*= SRCDIR "/output_calib.json"*/)
{
  decodeJson(params_, i_filepath);
  return params_;
}

bool CameraParamsJsonParser::decodeJson(
  CameraParams& params_out,
  const std::string& i_filepath /*= SRCDIR "/output_calib.json"*/)
{
  std::ifstream i_file(i_filepath);
  if (!i_file.is_open())
  {
    std::cerr << "[ERROR] Could not open file" << std::endl;
    return false;
  }
  if (isEmpty(i_file))
  {
    std::cerr << "[ERROR] Found an empty file. Camera parameters set as default"
              << std::endl;
    return false;
  }

  json parameters_2_read;
  i_file >> parameters_2_read;
  i_file.close();

  return decode(parameters_2_read, params_out);
}

bool CameraParamsJsonParser::decode(const json& container, CameraParams& params_out)
{
  // Extract camera matrix and run some safety checks.
  std::vector<double> camera_matrix =
    container["camera_matrix"].get<std::vector<double>>();
  for (uchar i = 0; i < camera_matrix.size(); i++)
    if (camera_matrix[i] < 0)
    {
      std::cerr << "[ERROR] Invalid camera matrix, all elements must be positive"
                << std::endl;
      return false;
    }
  if (camera_matrix[1] != 0.0 || camera_matrix[3] != 0.0 || camera_matrix[6] != 0.0 ||
      camera_matrix[7] != 0.0 || camera_matrix[8] != 1.0)
  {
    std::cerr << "[ERROR] Invalid camera matrix, wrong matrix definition" << std::endl;
    return false;
  }

  // Extract distortion coefficients.
  std::vector<double> dist_coeff = container["dist_coeff"].get<std::vector<double>>();

  // Fill camera parameters structure.
  params_out.fill(camera_matrix, dist_coeff);

  return true;
}

//--------- Private functions --------------------------------------------------------//

bool CameraParamsJsonParser::isEmpty(std::ifstream& p_file)
{
  return p_file.peek() == std::ifstream::traits_type::eof();
}
