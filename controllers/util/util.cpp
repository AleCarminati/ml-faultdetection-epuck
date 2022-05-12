#include "util.h"

bool CConfiguration::initialized = false;

/* Initialization of the static variables.*/
float CConfiguration::INTERWHEEL_DISTANCE = 0.0f;
float CConfiguration::HALF_INTERWHEEL_DISTANCE = 0.0f;
float CConfiguration::MAX_FORCE = 0.0f;
float CConfiguration::MASS = 0.0f;
float CConfiguration::ACCELERATION = 0.0f;
float CConfiguration::TIME_STEP = 0.0f;
float CConfiguration::TIME_STEP_PHYSICS = 0.0f;
std::string CConfiguration::FAULT_NONE = "";
std::string CConfiguration::FAULT_PMIN = "";
std::string CConfiguration::FAULT_PMAX = "";
std::string CConfiguration::FAULT_PRND = "";
std::string CConfiguration::FAULT_ROFS = "";
std::string CConfiguration::FAULT_LACT = "";
std::string CConfiguration::FAULT_RACT = "";
std::string CConfiguration::FAULT_BACT = "";
unsigned int CConfiguration::ID_FAULTY_ROBOT = 0u;
int CConfiguration::INVALID_DISPLACEMENT = 0;
float CConfiguration::INVALID_MIN_NEIGHBOR_DISTANCE = 0u;
int CConfiguration::INVALID_FAULT_PROBABILITY = 0;
float CConfiguration::INVALID_ANGULAR_SPEED = 0.0f;
bool CConfiguration::COLLISION_CONSERVATIVE = false;
bool CConfiguration::ENDOGENOUS_OBSERVATION = false;
bool CConfiguration::BOOLEAN_OBSERVATIONS = false;
float CConfiguration::PROBABILITY_THRESHOLD = 0.5f;
std::string CConfiguration::AGGREGATE = "";
std::string CConfiguration::DISPERSE = "";
std::string CConfiguration::FLOCKING = "";
std::string CConfiguration::HOMING = "";
CRadians CConfiguration::NO_TURN_THRESHOLD = CRadians();
CRadians CConfiguration::SOFT_TURN_THRESHOLD = CRadians();
int CConfiguration::NO_TURN = 0;
int CConfiguration::SOFT_TURN = 0;
int CConfiguration::HARD_TURN = 0;
float CConfiguration::RANGE_THRESHOLD_AGGREGATION = 0.0f;
CVector2 CConfiguration::INVALID_DISPLACEMENT_VECTOR = CVector2();
float CConfiguration::PROXIMITY_SENSOR_THRESHOLD_DISPERSION = 0.0f;
CRadians CConfiguration::GO_STRAIGHT_ANGLE_THRESHOLD_DISPERSION = CRadians();
float CConfiguration::CHANGE_DIRECTION_PROBABILITY_RANDOMWALK = 0.0f;
unsigned int CConfiguration::NOT_BEACON_MESSAGE = 0u;
unsigned int CConfiguration::BEACON_MESSAGE = 0u;
unsigned int CConfiguration::BEACON_ID = 0u;
unsigned int CConfiguration::BEACON_BYTE_INDEX = 0u;

/* Initialization of the source of randomness. Its seed will be based on the
 * seed selected in the .argos file.
 */
CRandom::CRNG* CSourceOfRandomness::m_pcRNG = CRandom::CreateRNG("argos");

const std::string CConfiguration::configurationFilePath = "config.ini";

void CConfiguration::ReadConfigurationFile() {
  std::ifstream fstrm(configurationFilePath);
  std::string line;
  unsigned lineNumber = 0u;

  if (!fstrm.is_open()) {
    THROW_ARGOSEXCEPTION("Error in opening the configuration file.");
  }

  while (!fstrm.eof()) {
    std::getline(fstrm, line);
    lineNumber++;

    // Remove all the whitespaces.
    line.erase(std::remove_if(line.begin(), line.end(), isspace), line.end());
    ;

    // Empty lines, sections and comments are not considered.
    if (!line.empty() && line[0] != '#' && line[0] != '[') {
      size_t equalIndex = line.find("=");
      if (equalIndex == std::string::npos) {
        THROW_ARGOSEXCEPTION("Missing '=' in line " +
                             std::to_string(lineNumber) + ".");
      }

      std::string leftSide = line.substr(0, equalIndex);
      std::string rightSide = line.substr(equalIndex + 1, std::string::npos);

      if (leftSide.compare("INTERWHEEL_DISTANCE") == 0) {
        INTERWHEEL_DISTANCE = std::stof(rightSide);
      } else if (leftSide.compare("MAX_FORCE") == 0) {
        MAX_FORCE = std::stof(rightSide);
      } else if (leftSide.compare("MASS") == 0) {
        MASS = std::stof(rightSide);
      } else if (leftSide.compare("TIME_STEP") == 0) {
        TIME_STEP = std::stof(rightSide);
      } else if (leftSide.compare("TIME_STEP_PHYSICS") == 0) {
        TIME_STEP_PHYSICS = std::stof(rightSide);
      } else if (leftSide.compare("FAULT_NONE") == 0) {
        FAULT_NONE.assign(rightSide);
      } else if (leftSide.compare("FAULT_PMIN") == 0) {
        FAULT_PMIN.assign(rightSide);
      } else if (leftSide.compare("FAULT_PMAX") == 0) {
        FAULT_PMAX.assign(rightSide);
      } else if (leftSide.compare("FAULT_PRND") == 0) {
        FAULT_PRND.assign(rightSide);
      } else if (leftSide.compare("FAULT_ROFS") == 0) {
        FAULT_ROFS.assign(rightSide);
      } else if (leftSide.compare("FAULT_LACT") == 0) {
        FAULT_LACT.assign(rightSide);
      } else if (leftSide.compare("FAULT_RACT") == 0) {
        FAULT_RACT.assign(rightSide);
      } else if (leftSide.compare("FAULT_BACT") == 0) {
        FAULT_BACT.assign(rightSide);
      } else if (leftSide.compare("ID_FAULTY_ROBOT") == 0) {
        ID_FAULTY_ROBOT = std::stoi(rightSide);
      } else if (leftSide.compare("INVALID_DISPLACEMENT") == 0) {
        INVALID_DISPLACEMENT = std::stoi(rightSide);
      } else if (leftSide.compare("INVALID_MIN_NEIGHBOR_DISTANCE") == 0) {
        INVALID_MIN_NEIGHBOR_DISTANCE = std::stoi(rightSide);
      } else if (leftSide.compare("INVALID_FAULT_PROBABILITY") == 0) {
        INVALID_FAULT_PROBABILITY = std::stoi(rightSide);
      } else if (leftSide.compare("INVALID_ANGULAR_SPEED") == 0) {
        INVALID_ANGULAR_SPEED = std::stof(rightSide);
      } else if (leftSide.compare("COLLISION_CONSERVATIVE") == 0) {
        if (rightSide.compare("true") == 0 || rightSide.compare("TRUE") == 0) {
          COLLISION_CONSERVATIVE = true;
        } else {
          COLLISION_CONSERVATIVE = false;
        }
      } else if (leftSide.compare("ENDOGENOUS_OBSERVATION") == 0) {
        if (rightSide.compare("true") == 0 || rightSide.compare("TRUE") == 0) {
          ENDOGENOUS_OBSERVATION = true;
        } else {
          ENDOGENOUS_OBSERVATION = false;
        }
      } else if (leftSide.compare("BOOLEAN_OBSERVATIONS") == 0) {
        if (rightSide.compare("true") == 0 || rightSide.compare("TRUE") == 0) {
          BOOLEAN_OBSERVATIONS = true;
        } else {
          BOOLEAN_OBSERVATIONS = false;
        }
      } else if (leftSide.compare("PROBABILITY_THRESHOLD") == 0) {
        PROBABILITY_THRESHOLD = std::stof(rightSide);
      } else if (leftSide.compare("AGGREGATE") == 0) {
        AGGREGATE.assign(rightSide);
      } else if (leftSide.compare("DISPERSE") == 0) {
        DISPERSE.assign(rightSide);
      } else if (leftSide.compare("FLOCKING") == 0) {
        FLOCKING.assign(rightSide);
      } else if (leftSide.compare("HOMING") == 0) {
        HOMING.assign(rightSide);
      } else if (leftSide.compare("NO_TURN_THRESHOLD") == 0) {
        int divisionIndex = rightSide.find("/");
        if (divisionIndex == std::string::npos) {
          THROW_ARGOSEXCEPTION("Missing '/' in line " +
                               std::to_string(lineNumber) + ".");
        }
        float numerator = std::stof(rightSide.substr(0, divisionIndex));
        float denominator =
            std::stof(rightSide.substr(divisionIndex + 1, std::string::npos));
        NO_TURN_THRESHOLD = CRadians::PI * numerator / denominator;
      } else if (leftSide.compare("SOFT_TURN_THRESHOLD") == 0) {
        int divisionIndex = rightSide.find("/");
        if (divisionIndex == std::string::npos) {
          THROW_ARGOSEXCEPTION("Missing '/' in line " +
                               std::to_string(lineNumber) + ".");
        }
        float numerator = std::stof(rightSide.substr(0, divisionIndex));
        float denominator =
            std::stof(rightSide.substr(divisionIndex + 1, std::string::npos));
        SOFT_TURN_THRESHOLD = CRadians::PI * numerator / denominator;
      } else if (leftSide.compare("NO_TURN") == 0) {
        NO_TURN = std::stoi(rightSide);
      } else if (leftSide.compare("SOFT_TURN") == 0) {
        SOFT_TURN = std::stoi(rightSide);
      } else if (leftSide.compare("HARD_TURN") == 0) {
        HARD_TURN = std::stoi(rightSide);
      } else if (leftSide.compare("RANGE_THRESHOLD_AGGREGATION") == 0) {
        RANGE_THRESHOLD_AGGREGATION = std::stoi(rightSide);
      } else if (leftSide.compare("INVALID_DISPLACEMENT_VECTOR") == 0) {
        int commaIndex = rightSide.find(",");
        if (commaIndex == std::string::npos) {
          THROW_ARGOSEXCEPTION("Missing ',' in line " +
                               std::to_string(lineNumber) + ".");
        }
        float x = std::stof(rightSide.substr(0, commaIndex));
        float y =
            std::stof(rightSide.substr(commaIndex + 1, std::string::npos));
        INVALID_DISPLACEMENT_VECTOR = CVector2(x, y);
      } else if (leftSide.compare("PROXIMITY_SENSOR_THRESHOLD_DISPERSION") ==
                 0) {
        PROXIMITY_SENSOR_THRESHOLD_DISPERSION = std::stof(rightSide);
      } else if (leftSide.compare("GO_STRAIGHT_ANGLE_THRESHOLD_DISPERSION") ==
                 0) {
        int divisionIndex = rightSide.find("/");
        if (divisionIndex == std::string::npos) {
          THROW_ARGOSEXCEPTION("Missing '/' in line " +
                               std::to_string(lineNumber) + ".");
        }
        float numerator = std::stof(rightSide.substr(0, divisionIndex));
        float denominator =
            std::stof(rightSide.substr(divisionIndex + 1, std::string::npos));
        GO_STRAIGHT_ANGLE_THRESHOLD_DISPERSION =
            CRadians::PI * numerator / denominator;
      } else if (leftSide.compare("CHANGE_DIRECTION_PROBABILITY_RANDOMWALK") ==
                 0) {
        CHANGE_DIRECTION_PROBABILITY_RANDOMWALK = std::stof(rightSide);
      } else if (leftSide.compare("NOT_BEACON_MESSAGE") == 0) {
        NOT_BEACON_MESSAGE = std::stoi(rightSide);
      } else if (leftSide.compare("BEACON_MESSAGE") == 0) {
        BEACON_MESSAGE = std::stoi(rightSide);
      } else if (leftSide.compare("BEACON_ID") == 0) {
        BEACON_ID = std::stoi(rightSide);
      } else if (leftSide.compare("BEACON_BYTE_INDEX") == 0) {
        BEACON_BYTE_INDEX = std::stoi(rightSide);
      } else {
        THROW_ARGOSEXCEPTION(
            "Unable to recognize the variable on the left side in line " +
            std::to_string(lineNumber) + ".");
      }
    }
  }

  /* Set the constants that can be derived from the ones read in the config
   * file. */
  HALF_INTERWHEEL_DISTANCE = INTERWHEEL_DISTANCE / 2.0f;
  ACCELERATION = MAX_FORCE * 100.0f / MASS;

  /* To check if the function parsed correctly the configuration file, uncomment
   * the next line.
   */
  // PrintConfiguration();

  initialized = true;

  fstrm.close();
};

void CConfiguration::PrintConfiguration() {
  std::string output = "";

  output += "[GENERAL]\n";
  output += "INTERWHEEL_DISTANCE = " +
            std::to_string(CConfiguration::INTERWHEEL_DISTANCE) + "\n";
  output += "HALF_INTERWHEEL_DISTANCE = " +
            std::to_string(CConfiguration::HALF_INTERWHEEL_DISTANCE) + "\n";
  output += "MAX_FORCE = " + std::to_string(CConfiguration::MAX_FORCE) + "\n";
  output += "MASS = " + std::to_string(CConfiguration::MASS) + "\n";
  output +=
      "ACCELERATION = " + std::to_string(CConfiguration::ACCELERATION) + "\n";
  output += "TIME_STEP = " + std::to_string(CConfiguration::TIME_STEP) + "\n";
  output += "TIME_STEP_PHYSICS = " +
            std::to_string(CConfiguration::TIME_STEP_PHYSICS) + "\n";

  output += "\n[FAULT]\n";
  output += "FAULT_NONE = " + CConfiguration::FAULT_NONE + "\n";
  output += "FAULT_PMIN = " + CConfiguration::FAULT_PMIN + "\n";
  output += "FAULT_PMAX = " + CConfiguration::FAULT_PMAX + "\n";
  output += "FAULT_PRND = " + CConfiguration::FAULT_PRND + "\n";
  output += "FAULT_ROFS = " + CConfiguration::FAULT_ROFS + "\n";
  output += "FAULT_LACT = " + CConfiguration::FAULT_LACT + "\n";
  output += "FAULT_RACT = " + CConfiguration::FAULT_RACT + "\n";
  output += "FAULT_BACT = " + CConfiguration::FAULT_BACT + "\n";
  output +=
      "ID_FAULTY_ROBOT = " + std::to_string(CConfiguration::ID_FAULTY_ROBOT) +
      "\n";

  output += "\n[OBSERVATION]\n";
  output += "INVALID_DISPLACEMENT = " +
            std::to_string(CConfiguration::INVALID_DISPLACEMENT) + "\n";
  output += "INVALID_MIN_NEIGHBOR_DISTANCE = " +
            std::to_string(CConfiguration::INVALID_MIN_NEIGHBOR_DISTANCE) +
            "\n";
  output += "INVALID_FAULT_PROBABILITY = " +
            std::to_string(CConfiguration::INVALID_FAULT_PROBABILITY) + "\n";
  output += "INVALID_ANGULAR_SPEED = " +
            std::to_string(CConfiguration::INVALID_ANGULAR_SPEED) + "\n";
  output += "COLLISION_CONSERVATIVE = " +
            std::to_string(CConfiguration::COLLISION_CONSERVATIVE) + "\n";
  output += "ENDOGENOUS_OBSERVATION = " +
            std::to_string(CConfiguration::ENDOGENOUS_OBSERVATION) + "\n";
  output += "BOOLEAN_OBSERVATIONS = " +
            std::to_string(CConfiguration::BOOLEAN_OBSERVATIONS) + "\n";
  output += "PROBABILITY_THRESHOLD = " +
            std::to_string(CConfiguration::PROBABILITY_THRESHOLD) + "\n";

  output += "\n[BEHAVIOR]\n";
  output += "AGGREGATE = " + CConfiguration::AGGREGATE + "\n";
  output += "DISPERSE = " + CConfiguration::DISPERSE + "\n";
  output += "FLOCKING = " + CConfiguration::FLOCKING + "\n";
  output += "HOMING = " + CConfiguration::HOMING + "\n";
  output += "NO_TURN_THRESHOLD = " +
            std::to_string(CConfiguration::NO_TURN_THRESHOLD.GetValue()) + "\n";
  output += "SOFT_TURN_THRESHOLD = " +
            std::to_string(CConfiguration::SOFT_TURN_THRESHOLD.GetValue()) +
            "\n";
  output += "NO_TURN = " + std::to_string(CConfiguration::NO_TURN) + "\n";
  output += "SOFT_TURN = " + std::to_string(CConfiguration::SOFT_TURN) + "\n";
  output += "HARD_TURN = " + std::to_string(CConfiguration::HARD_TURN) + "\n";
  output += "RANGE_THRESHOLD_AGGREGATION = " +
            std::to_string(CConfiguration::RANGE_THRESHOLD_AGGREGATION) + "\n";
  output += "INVALID_DISPLACEMENT_VECTOR = (" +
            std::to_string(CConfiguration::INVALID_DISPLACEMENT_VECTOR.GetX()) +
            "," +
            std::to_string(CConfiguration::INVALID_DISPLACEMENT_VECTOR.GetY()) +
            ")\n";
  output +=
      "PROXIMITY_SENSOR_THRESHOLD_DISPERSION = " +
      std::to_string(CConfiguration::PROXIMITY_SENSOR_THRESHOLD_DISPERSION) +
      "\n";
  output +=
      "GO_STRAIGHT_ANGLE_THRESHOLD_DISPERSION = " +
      std::to_string(
          CConfiguration::GO_STRAIGHT_ANGLE_THRESHOLD_DISPERSION.GetValue()) +
      "\n";
  output +=
      "CHANGE_DIRECTION_PROBABILITY_RANDOMWALK = " +
      std::to_string(CConfiguration::CHANGE_DIRECTION_PROBABILITY_RANDOMWALK) +
      "\n";
  output += "NOT_BEACON_MESSAGE = " +
            std::to_string(CConfiguration::NOT_BEACON_MESSAGE) + "\n";
  output +=
      "BEACON_MESSAGE = " + std::to_string(CConfiguration::BEACON_MESSAGE) +
      "\n";
  output += "BEACON_ID = " + std::to_string(CConfiguration::BEACON_ID) + "\n";
  output += "BEACON_BYTE_INDEX = " +
            std::to_string(CConfiguration::BEACON_BYTE_INDEX) + "\n";

  std::cout << output << std::endl;
}

void computeNeighborsMatrix(CCI_RangeAndBearingSensor::TReadings& tRABReads,
                            NeighborAccumulator* matrix,
                            NeighborAccumulator* accumulators,
                            unsigned int TIME_WINDOW, unsigned int time,
                            std::string id_str, bool beacon) {
  /* Lambda to get the address of the i-th row. The explanation of why it is
  used can be found in the header file. */
  auto getRow = [matrix, TIME_WINDOW](int i) {
    return (NeighborAccumulator*)(matrix + i * TIME_WINDOW);
  };

  bool observed[N_ROBOTS] = {false};

  for (int i = 0; i < tRABReads.size(); ++i) {
    UInt8 id;
    float observedVelLeftWheel;
    float observedVelRightWheel;
    UInt8 beaconByte;
    CByteArray message = CByteArray(tRABReads[i].Data);
    message >> id;
    message >> observedVelLeftWheel;
    message >> observedVelRightWheel;
    message >> beaconByte;

    /* If the observed robot is the beacon, do not compute its neighbors. */
    if (beaconByte != CConfiguration::BEACON_MESSAGE) {
      bool innerRange = false;
      bool outerRange = false;
      for (int j = 0; j < tRABReads.size(); ++j) {
        float observedNeighborDistance;

        if (j != i && tRABReads[j].Data[CConfiguration::BEACON_BYTE_INDEX] ==
                          CConfiguration::NOT_BEACON_MESSAGE) {
          observedNeighborDistance =
              sqrt(pow(tRABReads[i].Range, 2) + pow(tRABReads[j].Range, 2) -
                   2 * tRABReads[i].Range * tRABReads[j].Range *
                       Cos(tRABReads[i].HorizontalBearing -
                           tRABReads[j].HorizontalBearing));
          if (observedNeighborDistance < 15) {
            innerRange = true;
          } else if (observedNeighborDistance < 30) {
            outerRange = true;
          }
        }
      }
      /* Also the observer is a neighbor.*/
      if (!beacon) {
        if (tRABReads[i].Range < 15) {
          innerRange = true;
        } else if (tRABReads[i].Range < 30) {
          outerRange = true;
        }
      }
      if (innerRange) {
        for (int k = 0; k < TIME_WINDOW; ++k) {
          getRow(id)[k].neighborInnerRangeAccumulator++;
        }
      }
      if (outerRange) {
        for (int k = 0; k < TIME_WINDOW; ++k) {
          getRow(id)[k].neighborOuterRangeAccumulator++;
        }
      }
      observed[id] = true;
    }
  }

  /* If the robot observes itself, it computes the distribution of its
   * neighborhood using the RAB sensor. This time the law of cosines is not
   * used.
   */
  if (CConfiguration::ENDOGENOUS_OBSERVATION && !beacon) {
    UInt8 id;
    float observedVelLeftWheel;
    float observedVelRightWheel;
    UInt8 beaconByte;
    id_str.erase(0, 2);
    UInt8 id_int = std::stoi(id_str);
    bool innerRange = false;
    bool outerRange = false;
    for (int i = 0; i < tRABReads.size(); ++i) {
      CByteArray message = CByteArray(tRABReads[i].Data);
      message >> id;
      message >> observedVelLeftWheel;
      message >> observedVelRightWheel;
      message >> beaconByte;
      if (beaconByte != CConfiguration::BEACON_MESSAGE) {
        if (tRABReads[i].Range < 15) {
          innerRange = true;
        } else if (tRABReads[i].Range < 30) {
          outerRange = true;
        }
      }
    }
    if (innerRange) {
      for (int j = 0; j < TIME_WINDOW; ++j) {
        getRow(id_int)[j].neighborInnerRangeAccumulator++;
      }
    }
    if (outerRange) {
      for (int j = 0; j < TIME_WINDOW; ++j) {
        getRow(id_int)[j].neighborOuterRangeAccumulator++;
      }
    }
    observed[id_int] = true;
  }

  for (int i = 0; i < N_ROBOTS; ++i) {
    if (!observed[i]) {
      for (int j = 0; j < TIME_WINDOW; ++j) {
        getRow(i)[j].validity = false;
      }
    }
  }

  for (int i = 0; i < N_ROBOTS; ++i) {
    accumulators[i] = {
        getRow(i)[time % TIME_WINDOW].neighborInnerRangeAccumulator,
        getRow(i)[time % TIME_WINDOW].neighborOuterRangeAccumulator,
        getRow(i)[time % TIME_WINDOW].validity};
    getRow(i)[time % TIME_WINDOW] = {0u, 0u, true};
  }
}

void computeDisplacementsMatrix(
    CCI_RangeAndBearingSensor::TReadings& tRABReads,
    const CCI_DifferentialSteeringSensor::SReading& tDifReads,
    float* lastWheelVel, DisplacementAccumulator* matrix,
    CVector2* displacements, unsigned int TIME_WINDOW, unsigned int time,
    std::string id_str) {
  float deltaX = 0.0f;
  float deltaY = 0.0f;
  float deltaOmega = 0.0f;

  /* Lambda to get the address of the i-th row. The explanation of why it is
  used can be found in the header file. */
  auto getRow = [matrix, TIME_WINDOW](int i) {
    return (DisplacementAccumulator*)(matrix + i * TIME_WINDOW);
  };

  /* Update transitions and rotations of the observer robot. */
  for (size_t i = 0; i < N_ROBOTS; ++i) {
    for (int j = 0; j < TIME_WINDOW; ++j) {
      if (getRow(i)[j].validity) {
        deltaOmega = 0.0f;
        deltaX = 0.0f;
        deltaY = 0.0f;
        computeOwnMovement(tDifReads.VelocityLeftWheel,
                           tDifReads.VelocityRightWheel, lastWheelVel[0],
                           lastWheelVel[1],
                           getRow(i)[j].observerRotation.GetValue(), deltaOmega,
                           deltaX, deltaY);
        getRow(i)[j].observerRotation += CRadians(deltaOmega);
        getRow(i)[j].observerTranslation += CVector2(deltaX, deltaY);
      }
    }
  }

  bool observed[N_ROBOTS] = {false};
  UInt8 id;

  for (int i = 0; i < tRABReads.size(); ++i) {
    CByteArray(tRABReads[i].Data) >> id;

    // Compute the displacement.
    if (getRow(id)[time % TIME_WINDOW].validity) {
      /* The function Rotate() of ARGoS's CVector2 implements the function:
       * x_new = x_old * cos(deltaOmega) - y_old * sin(deltaOmega)
       * y_new = x_old * sin(deltaOmega) + y_old * cos(deltaOmega)
       */
      CVector2 newPosition =
          CVector2(tRABReads[i].Range, tRABReads[i].HorizontalBearing)
              .Rotate(getRow(id)[time % TIME_WINDOW].observerRotation) +
          getRow(id)[time % TIME_WINDOW].observerTranslation;

      displacements[id] =
          newPosition - getRow(id)[time % TIME_WINDOW].initialPosition;
    } else {
      displacements[id] = (CConfiguration::INVALID_DISPLACEMENT_VECTOR);
    }

    // Create a new DisplacementAccumulator.
    getRow(id)[time % TIME_WINDOW] = {
        CVector2(tRABReads[i].Range, tRABReads[i].HorizontalBearing),
        CVector2(), CRadians(), true};
    observed[id] = true;
  }

  /* If the feature is active in the configuration, the robot observes
   * itself.
   */
  if (CConfiguration::ENDOGENOUS_OBSERVATION) {
    id_str.erase(0, 2);
    UInt8 id_int = std::stoi(id_str);

    displacements[id_int] =
        CVector2(getRow(id_int)[time % TIME_WINDOW].observerTranslation.GetX(),
                 getRow(id_int)[time % TIME_WINDOW].observerTranslation.GetY());

    getRow(id_int)[time % TIME_WINDOW] = {CVector2(), CVector2(), CRadians(),
                                          true};

    observed[id_int] = true;
  }

  // Create records for all the robots that haven't been observed in this
  // control step.
  for (int i = 0; i < N_ROBOTS; ++i) {
    if (!observed[i]) {
      displacements[i] = (CConfiguration::INVALID_DISPLACEMENT_VECTOR);

      getRow(i)[time % TIME_WINDOW] = {CVector2(), CVector2(), CRadians(),
                                       false};
    }
  }
}

void computeOwnMovement(float velLeftWheel, float velRightWheel,
                        float lastVelLeftWheel, float lastVelRightWheel,
                        float initialOmega, float& deltaOmega, float& deltaX,
                        float& deltaY) {
  double timeStep = CConfiguration::TIME_STEP;
  float deltaOmegaTemp;
  deltaOmega = 0;

  /* Compute measures for transient.
   * One main assumption: the transient time is lower or equal to the time step.
   */
  if (lastVelLeftWheel != velLeftWheel || lastVelRightWheel != velRightWheel) {
    float velLeftWheelTemp = lastVelLeftWheel;
    float velRightWheelTemp = lastVelRightWheel;
    float accelerationLeftWheel = 0.0f;
    float accelerationRightWheel = 0.0f;

    /* Compute time acceleration applied to the wheels. */
    if (lastVelLeftWheel != velLeftWheel) {
      accelerationLeftWheel = (velLeftWheel - lastVelLeftWheel) /
                              fabsf(velLeftWheel - lastVelLeftWheel) *
                              (CConfiguration::ACCELERATION);
    }
    if (lastVelRightWheel != velRightWheel) {
      accelerationRightWheel = (velRightWheel - lastVelRightWheel) /
                               fabsf(velRightWheel - lastVelRightWheel) *
                               (CConfiguration::ACCELERATION);
    }

    while ((velLeftWheelTemp != velLeftWheel ||
            velRightWheelTemp != velRightWheel) &&
           timeStep > 0) {
      timeStep -= CConfiguration::TIME_STEP_PHYSICS;
      deltaOmegaTemp = (-velLeftWheelTemp + velRightWheelTemp) *
                       CConfiguration::TIME_STEP_PHYSICS /
                       CConfiguration::INTERWHEEL_DISTANCE;

      /* initialOmega + deltaOmega is the rotation at the start of the physics
       * simulation cycle, while deltaOmegaTemp is the rotation in the last
       * cycle.
       */
      deltaX += (velLeftWheelTemp + velRightWheelTemp) *
                CConfiguration::TIME_STEP_PHYSICS *
                cos(initialOmega + deltaOmega + deltaOmegaTemp / 2) / 2;
      deltaY += (velLeftWheelTemp + velRightWheelTemp) *
                CConfiguration::TIME_STEP_PHYSICS *
                sin(initialOmega + deltaOmega + deltaOmegaTemp / 2) / 2;

      deltaOmega += deltaOmegaTemp;

      if (velLeftWheelTemp > velLeftWheel) {
        velLeftWheelTemp =
            fmax(velLeftWheel,
                 velLeftWheelTemp +
                     accelerationLeftWheel * CConfiguration::TIME_STEP_PHYSICS);
      } else if (velLeftWheelTemp < velLeftWheel) {
        velLeftWheelTemp =
            fmin(velLeftWheel,
                 velLeftWheelTemp +
                     accelerationLeftWheel * CConfiguration::TIME_STEP_PHYSICS);
      }

      if (velRightWheelTemp > velRightWheel) {
        velRightWheelTemp =
            fmax(velRightWheel,
                 velRightWheelTemp + accelerationRightWheel *
                                         CConfiguration::TIME_STEP_PHYSICS);
      } else if (velRightWheelTemp < velRightWheel) {
        velRightWheelTemp =
            fmin(velRightWheel,
                 velRightWheelTemp + accelerationRightWheel *
                                         CConfiguration::TIME_STEP_PHYSICS);
      }
    }
  }

  deltaOmegaTemp = (-velLeftWheel + velRightWheel) * timeStep /
                   CConfiguration::INTERWHEEL_DISTANCE;
  deltaX += (velLeftWheel + velRightWheel) * timeStep *
            cos(initialOmega + deltaOmega + deltaOmegaTemp / 2) / 2;
  deltaY += (velLeftWheel + velRightWheel) * timeStep *
            sin(initialOmega + deltaOmega + deltaOmegaTemp / 2) / 2;
  deltaOmega += deltaOmegaTemp;
}

unsigned int stringReplace(std::string& string, char characterToFind,
                           char characterToReplace) {
  unsigned int count = 0u;

  size_t firstOccurrence = 0;

  firstOccurrence = string.find(characterToFind);

  while (firstOccurrence != std::string::npos) {
    string[firstOccurrence] = characterToReplace;
    firstOccurrence = string.find(characterToFind, firstOccurrence);
  }

  return count;
}
