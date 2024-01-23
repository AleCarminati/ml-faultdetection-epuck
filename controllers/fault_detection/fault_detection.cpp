/* Include the controller definition */
#include "fault_detection.h"
/* Function definitions for XML parsing */
#include <argos3/core/utility/configuration/argos_configuration.h>
/* Logging */
#include <argos3/core/utility/logging/argos_log.h>
#include <argos3/plugins/robots/e-puck/simulator/epuck_entity.h>

#include <algorithm>
#include <iostream>
#include <iterator>
#include <random>
#include <vector>

/****************************************/
/****************************************/

/*
 * UTILITIES
 *
 * This section contains some utility functions.
 */

/* This function pushes the last numerical observation at the tail of the array
 * and shifts all the other elements.
 */
void pushNumericalObservation(NumericalObservation array[],
                              NumericalObservation element) {
  for (int i = 0; i < N_OBSERVATIONS - 1; ++i) {
    array[i] = array[i + 1];
  }
  array[N_OBSERVATIONS - 1] = element;
}

/* This function pushes the last boolean observation at the tail of the array
 * and shifts all the other elements.
 */
void pushBooleanObservation(BooleanObservation array[],
                            BooleanObservation element) {
  for (int i = 0; i < N_OBSERVATIONS - 1; ++i) {
    array[i] = array[i + 1];
  }
  array[N_OBSERVATIONS - 1] = element;
}

#define safe_xgboost(call)                                                    \
  {                                                                           \
    int err = (call);                                                         \
    if (err != 0) {                                                           \
      THROW_ARGOSEXCEPTION(std::string(__FILE__) + ":" +                      \
                           std::to_string(__LINE__) + ": error in " + #call + \
                           ":" + XGBGetLastError());                          \
    }                                                                         \
  }

/****************************************/
/****************************************/

CEPuckFaultDetection::CEPuckFaultDetection()
    : m_pcWheels(NULL),
      m_pcWheelsSens(NULL),
      m_pcProximity(NULL),
      m_pcRABA(NULL),
      m_pcRABS(NULL),
      m_fWheelVelocity(2.5f),
      m_behavior(),
      m_flockingBehavior(NULL),
      m_numericalObservations(),
      m_booleanObservations(),
      m_faultProbabilities(),
      m_neighborsMatrix(),
      m_firstStep(true),
      m_lastWheelVel(),
      m_faultType(),
      m_training(true),
      m_consecutive_collisions(0),
      m_beacon(false),
      m_nearestProx(0.0f),
      m_timeCounter(0u),
      m_displacementsMatrix(),
      m_irrespPriorAlpha(),
      m_irrespPriorBeta(),
      m_noSensPriorAlpha(),
      m_noSensPriorBeta(),
      m_sensPriorAlpha(),
      m_sensPriorBeta(),
      m_displacementsMatrixBayes(),
      m_prevAngularSpeed(),
      m_faultProbabilityAccumulator(),
      m_localCoalitionList(),
      m_votesAccumulator(),
      booster() {}

/****************************************/
/****************************************/

void CEPuckFaultDetection::Init(TConfigurationNode& t_node) {
  /*
   * Get sensor/actuator handles
   *
   * The passed string (ex. "differential_steering") corresponds to the
   * XML tag of the device whose handle we want to have. For a list of
   * allowed values, type at the command prompt:
   *
   * $ argos3 -q actuators
   *
   * to have a list of all the possible actuators, or
   *
   * $ argos3 -q sensors
   *
   * to have a list of all the possible sensors.
   *
   * NOTE: ARGoS creates and initializes actuators and sensors
   * internally, on the basis of the lists provided the configuration
   * file at the <controllers><footbot_diffusion><actuators> and
   * <controllers><footbot_diffusion><sensors> sections. If you forgot to
   * list a device in the XML and then you request it here, an error
   * occurs.
   */
  m_pcWheels =
      GetActuator<CCI_DifferentialSteeringActuator>("differential_steering");
  m_pcWheelsSens =
      GetSensor<CCI_DifferentialSteeringSensor>("differential_steering");
  m_pcProximity = GetSensor<CCI_EPuckProximitySensor>("epuck_proximity");
  m_pcRABA = GetActuator<CCI_RangeAndBearingActuator>("range_and_bearing");
  m_pcRABS = GetSensor<CCI_RangeAndBearingSensor>("range_and_bearing");
  m_pcPosSens = GetSensor<CCI_PositioningSensor>("positioning");
  /*
   * Parse the parameters in .argos file.
   */
  GetNodeAttributeOrDefault(t_node, "fault_type", m_faultType, m_faultType);
  GetNodeAttributeOrDefault(t_node, "training", m_training, m_training);
  GetNodeAttributeOrDefault(t_node, "velocity", m_fWheelVelocity,
                            m_fWheelVelocity);
  GetNodeAttributeOrDefault(t_node, "behavior", m_behavior_str, m_behavior_str);

  // Parse the configuration file.
  if (!CConfiguration::initialized) {
    CConfiguration::ReadConfigurationFile();
  }

  /* The order of the behaviors is taken from Tarapore_PLOSONE_2017, more
   * precisely from epuck_hom_swarm.cpp. */
  if (m_behavior_str.compare(CConfiguration::AGGREGATE) == 0) {
    m_behavior = std::vector<std::shared_ptr<CBehavior>>();
    m_behavior.push_back(
        std::shared_ptr<CBehavior>(new CDisperseBehavior(m_fWheelVelocity)));
    m_behavior.push_back(
        std::shared_ptr<CBehavior>(new CAggregateBehavior(m_fWheelVelocity)));
    m_behavior.push_back(
        std::shared_ptr<CBehavior>(new CRandomWalkBehavior(m_fWheelVelocity)));
  } else if (m_behavior_str.compare(CConfiguration::DISPERSE) == 0) {
    m_behavior = std::vector<std::shared_ptr<CBehavior>>();
    m_behavior.push_back(
        std::shared_ptr<CBehavior>(new CDisperseBehavior(m_fWheelVelocity)));
    m_behavior.push_back(
        std::shared_ptr<CBehavior>(new CRandomWalkBehavior(m_fWheelVelocity)));
  } else if (m_behavior_str.compare(CConfiguration::FLOCKING) == 0) {
    m_flockingBehavior = std::make_shared<CFlockingBehavior>(m_fWheelVelocity);
    m_behavior = std::vector<std::shared_ptr<CBehavior>>();
    m_behavior.push_back(
        std::shared_ptr<CBehavior>(new CDisperseBehavior(m_fWheelVelocity)));
    m_behavior.push_back(std::shared_ptr<CBehavior>(m_flockingBehavior));
    m_behavior.push_back(
        std::shared_ptr<CBehavior>(new CRandomWalkBehavior(m_fWheelVelocity)));
  } else if (m_behavior_str.compare(CConfiguration::HOMING) == 0) {
    m_behavior = std::vector<std::shared_ptr<CBehavior>>();
    std::string id_str = CCI_Controller::GetId();
    id_str.erase(0, 2);
    if (std::stoi(id_str) != CConfiguration::BEACON_ID) {
      m_behavior.push_back(
          std::shared_ptr<CBehavior>(new CDisperseBehavior(m_fWheelVelocity)));
      m_behavior.push_back(
          std::shared_ptr<CBehavior>(new CHomingBehavior(m_fWheelVelocity)));
      m_behavior.push_back(std::shared_ptr<CBehavior>(
          new CRandomWalkBehavior(m_fWheelVelocity)));
    } else {
      m_beacon = true;
    }
  } else {
    THROW_ARGOSEXCEPTION(
        "Wrong value for parameter 'behavior' in .argos file.");
  }

  m_pcRABA->ClearData();    // <-Added this line to make it work
  m_pcRABA->SetData(0, 0);  // <-Added this line to make it work

  /* Allocate the memory needed to store the past observations of the
   * other robots. Initialize every cell making them invalid.
   */
  if (CConfiguration::BOOLEAN_OBSERVATIONS) {
    BooleanObservation observations[N_ROBOTS][N_OBSERVATIONS];
    memcpy(m_booleanObservations, observations,
           sizeof(BooleanObservation) * N_ROBOTS * N_OBSERVATIONS);
    for (int i = 0; i < N_ROBOTS; ++i) {
      for (int j = 0; j < N_OBSERVATIONS; ++j) {
        m_booleanObservations[i][j].validity = false;
      }
    }
  } else {
    NumericalObservation observations[N_ROBOTS][N_OBSERVATIONS];
    memcpy(m_numericalObservations, observations,
           sizeof(NumericalObservation) * N_ROBOTS * N_OBSERVATIONS);
    for (int i = 0; i < N_ROBOTS; ++i) {
      for (int j = 0; j < N_OBSERVATIONS; ++j) {
        m_numericalObservations[i][j].validity = false;
      }
    }
  }

  /* Allocate the memory needed to store the fault probabilities of the other
   * robots. Initialize every cell with a value that represents that the
   * classification is invalid. */
  float faultProbabilites[N_ROBOTS];
  memcpy(m_faultProbabilities, faultProbabilites, sizeof(float) * N_ROBOTS);
  /* If we are in generating training data, the fault probabilities are fixed.
   */
  if (m_training) {
    for (int i = 0; i < N_ROBOTS; ++i) {
      if (!check_fault(i, CConfiguration::FAULT_NONE)) {
        m_faultProbabilities[i] = 1.0f;
      } else {
        m_faultProbabilities[i] = 0.0f;
      }
    }
  } else {
    for (int i = 0; i < N_ROBOTS; ++i) {
      m_faultProbabilities[i] = CConfiguration::INVALID_FAULT_PROBABILITY;
    }
  }

  DisplacementAccumulator displacementsMatrix[N_ROBOTS]
                                             [TIME_WINDOW_OBSERVATION];
  memcpy(m_displacementsMatrix, displacementsMatrix,
         sizeof(DisplacementAccumulator) * N_ROBOTS * TIME_WINDOW_OBSERVATION);
  for (int i = 0; i < N_ROBOTS; ++i) {
    for (int j = 0; j < TIME_WINDOW_OBSERVATION; ++j) {
      m_displacementsMatrix[i][j].validity = false;
    }
  }

  if (CConfiguration::BOOLEAN_OBSERVATIONS) {
    NeighborAccumulator neighborsMatrix[N_ROBOTS][TIME_WINDOW_OBSERVATION];
    memcpy(m_neighborsMatrix, neighborsMatrix,
           sizeof(NeighborAccumulator) * N_ROBOTS * TIME_WINDOW_OBSERVATION);
    for (int i = 0; i < N_ROBOTS; ++i) {
      for (int j = 0; j < TIME_WINDOW_OBSERVATION; ++j) {
        m_neighborsMatrix[i][j].validity = false;
      }
    }
  }

  if (CConfiguration::BOOLEAN_OBSERVATIONS) {
    unsigned int irrespPriorAlpha[N_ROBOTS];
    memcpy(m_irrespPriorAlpha, irrespPriorAlpha,
           sizeof(unsigned int) * N_ROBOTS);
    for (int i = 0; i < N_ROBOTS; ++i) {
      m_irrespPriorAlpha[i] = 1u;
    }
    unsigned int irrespPriorBeta[N_ROBOTS];
    memcpy(m_irrespPriorBeta, irrespPriorBeta, sizeof(unsigned int) * N_ROBOTS);
    for (int i = 0; i < N_ROBOTS; ++i) {
      m_irrespPriorBeta[i] = 1u;
    }
    unsigned int noSensPriorAlpha[N_ROBOTS];
    memcpy(m_noSensPriorAlpha, noSensPriorAlpha,
           sizeof(unsigned int) * N_ROBOTS);
    for (int i = 0; i < N_ROBOTS; ++i) {
      m_noSensPriorAlpha[i] = 1u;
    }
    unsigned int noSensPriorBeta[N_ROBOTS];
    memcpy(m_noSensPriorBeta, noSensPriorBeta, sizeof(unsigned int) * N_ROBOTS);
    for (int i = 0; i < N_ROBOTS; ++i) {
      m_noSensPriorBeta[i] = 1u;
    }
    unsigned int sensPriorAlpha[N_ROBOTS];
    memcpy(m_sensPriorAlpha, sensPriorAlpha, sizeof(unsigned int) * N_ROBOTS);
    for (int i = 0; i < N_ROBOTS; ++i) {
      m_sensPriorAlpha[i] = 1u;
    }
    unsigned int sensPriorBeta[N_ROBOTS];
    memcpy(m_sensPriorBeta, sensPriorBeta, sizeof(unsigned int) * N_ROBOTS);
    for (int i = 0; i < N_ROBOTS; ++i) {
      m_sensPriorBeta[i] = 1u;
    }
    DisplacementAccumulator displacementsMatrixBayes[N_ROBOTS]
                                                    [TIME_WINDOW_BAYES];
    memcpy(m_displacementsMatrixBayes, displacementsMatrixBayes,
           sizeof(DisplacementAccumulator) * N_ROBOTS * TIME_WINDOW_BAYES);
    for (int i = 0; i < N_ROBOTS; ++i) {
      for (int j = 0; j < TIME_WINDOW_BAYES; ++j) {
        m_displacementsMatrixBayes[i][j].validity = false;
      }
    }
    float prevAngularSpeed[N_ROBOTS];
    memcpy(m_prevAngularSpeed, prevAngularSpeed, sizeof(float) * N_ROBOTS);
    for (int i = 0; i < N_ROBOTS; ++i) {
      m_prevAngularSpeed[i] = CConfiguration::INVALID_ANGULAR_SPEED;
    }
  }

  FaultProbabilityAccumulator faultProbabilityAccumulator[N_ROBOTS];
  memcpy(m_faultProbabilityAccumulator, faultProbabilityAccumulator,
         sizeof(FaultProbabilityAccumulator) * N_ROBOTS);
  for (int i = 0; i < N_ROBOTS; ++i) {
    m_faultProbabilityAccumulator[i].faultProbabilityAccumulator = 0.0f;
    m_faultProbabilityAccumulator[i].numberOfProbabilites = 0u;
  }

  if (!m_training) {
    LocalCoalition localCoalitionList[N_ROBOTS];
    memcpy(m_localCoalitionList, localCoalitionList,
           sizeof(LocalCoalition) * N_ROBOTS);
    for (int i = 0; i < N_ROBOTS; ++i) {
      m_localCoalitionList[i].validity = false;
    }

    VotesAccumulator votesAccumulator[N_ROBOTS];
    memcpy(m_votesAccumulator, votesAccumulator,
           sizeof(VotesAccumulator) * N_ROBOTS);
    for (int i = 0; i < N_ROBOTS; ++i) {
      m_votesAccumulator[i].idVotersList.clear();
      m_votesAccumulator[i].numNormalVotes = 0u;
      m_votesAccumulator[i].numFaultyVotes = 0u;
    }
  }

  /* Initialize the vector that contains the speed of the wheels in the last
   * step. */
  m_lastWheelVel[0] = 0.0f;
  m_lastWheelVel[1] = 0.0f;

  /* Initialize XGBoost. */
  if (!m_training && use_xg_booster()) {
    safe_xgboost(XGBoosterCreate(0, 0, &booster));
    std::string path = "./classifiers/booster_";
    path += (m_behavior_str + "_");
    if (CConfiguration::BOOLEAN_OBSERVATIONS) {
      path += "bin";
    } else {
      path += "num";
    }
    path += ".json";
    safe_xgboost(XGBoosterLoadModel(booster, path.c_str()));
  }

  std::string filename = generate_output_filename();

  bool newFile = false;
  std::ifstream fileExistence(filename);
  if (!fileExistence) {
    newFile = true;
  }
  fileExistence.close();

  output_file.open(filename, std::ofstream::out | std::ofstream::app);

  if (!output_file.is_open()) {
    THROW_ARGOSEXCEPTION("Error in opening the output CSV file.");
  }

  // If the file has just been created, write its header.
  if (newFile) {
    if (m_training) {
      output_file << "id_experiment;control_step;observed_robot;";
      if (CConfiguration::BOOLEAN_OBSERVATIONS) {
        for (int i = N_OBSERVATIONS - 1; i >= 0; --i) {
          for (int j = 1; j < 7; ++j) {
            output_file << "F" + std::to_string(j) + "_t-" + std::to_string(i) +
                               ";";
          }
        }
      } else {
        for (int i = N_OBSERVATIONS - 1; i >= 0; --i) {
          output_file << "vel_left_wheel_t-" + std::to_string(i) + ";";
          output_file << "vel_right_wheel_t-" + std::to_string(i) + ";";
          output_file << "distance_traveled_t-" + std::to_string(i) + ";";
          output_file << "min_neighbor_distance_t-" + std::to_string(i) + ";";
          output_file << "avg_neighbor_distance_t-" + std::to_string(i) + ";";
        }
      }
      output_file << "fault_probability" << std::endl;
    } else {
      output_file
          << "control_step;observed_robot;number_votes;coalition_response;";
      output_file << "real_faulty" << std::endl;
    }
  }
}

/****************************************/
/****************************************/

void CEPuckFaultDetection::ControlStep() {
  m_pcRABA
      ->ClearData();  // Clear the channel at the start of each control cycle.

  /* Detect the id of the robot. */
  std::string id_str = CCI_Controller::GetId();
  id_str.erase(0, 2);
  UInt8 id_int = std::stoi(id_str);

  /* Get readings from differentiate steering sensor */
  const CCI_DifferentialSteeringSensor::SReading tDifReads =
      m_pcWheelsSens->GetReading();

  /* Get readings from proximity sensor. Correct them if the robot is faulty. */
  CCI_EPuckProximitySensor::TReadings tProxReads;
  GetAndCorrectProxReadings(tProxReads);

  /* Get readings from RAB sensor. Correct them if the robot is faulty. */
  CCI_RangeAndBearingSensor::TReadings tRABReads;
  GetAndCorrectRABReadings(tRABReads);

  // Compare your results with the ones receive from the neighbors.
  if (CConfiguration::BOOLEAN_OBSERVATIONS) {
    CompareObservations(tRABReads);
  }

  // If we are not in training, use the classification algorithm.
  if (!m_training) {
    /* An entire round of observing and coalition formation finished.
     * Write the results in a .csv file.
     */
    if (m_timeCounter % (COMPARISON_CONTROL_STEPS + OBSERVE_CONTROL_STEPS) ==
        0) {
      WriteToCSVTesting();
      // Reset the probability accumulators.
      for (int i = 0; i < N_ROBOTS; ++i) {
        m_faultProbabilityAccumulator[i].faultProbabilityAccumulator = 0.0f;
        m_faultProbabilityAccumulator[i].numberOfProbabilites = 0u;
      }
      // Reset the structures used in coalition formation.
      for (int i = 0; i < N_ROBOTS; ++i) {
        m_votesAccumulator[i].idVotersList.clear();
        m_votesAccumulator[i].numNormalVotes = 0u;
        m_votesAccumulator[i].numFaultyVotes = 0u;
        m_localCoalitionList[i].validity = false;
      }
    } else if (m_timeCounter %
                   (COMPARISON_CONTROL_STEPS + OBSERVE_CONTROL_STEPS) <=
               OBSERVE_CONTROL_STEPS) {
      UpdateFaultProbabilities();
    } else if (m_timeCounter %
                   (COMPARISON_CONTROL_STEPS + OBSERVE_CONTROL_STEPS) >
               OBSERVE_CONTROL_STEPS) {
      ReadMessagesCoalitionFormation(tRABReads);
    }
  }

  if (m_training && check_fault(id_int, CConfiguration::FAULT_NONE)) {
    WriteToCSVTraining();
  }

  /*if(m_training && id_int == 15){
     WriteToCSVTraining();
  }*/

  if (CConfiguration::BOOLEAN_OBSERVATIONS) {
    ObserveNeighborsBoolean(tRABReads, tDifReads, tProxReads);
  } else {
    ObserveNeighborsNumerical(tRABReads, tDifReads, tProxReads);
  }

  /*m_lastPosVector.push_back(m_pcPosSens->GetReading());
  if(m_timeCounter >= TIME_WINDOW_OBSERVATION){
     m_lastPosVector.erase(m_lastPosVector.begin());
  }

  std::ofstream fstrm("0.01_noise_integration.csv", std::ofstream::app);
  std::string output;
  if(id_int==19){
     if(m_timeCounter >= TIME_WINDOW_OBSERVATION){
        output =
  std::to_string(CVector2(m_lastPosVector[0].Position.GetX()*100-m_lastPosVector[TIME_WINDOW_OBSERVATION-1].Position.GetX()*100,
  m_lastPosVector[0].Position.GetY()*100-m_lastPosVector[TIME_WINDOW_OBSERVATION-1].Position.GetY()*100).Length());
     }else{
        output = "";
     }
     output+="\n";
  }else{
     if(m_numericalObservations[19][N_OBSERVATIONS-1].validity &&
  m_numericalObservations[19][N_OBSERVATIONS-1].displacement!=CConfiguration::INVALID_DISPLACEMENT){
        output =
  std::to_string(m_numericalObservations[19][N_OBSERVATIONS-1].displacement);
     }else{
        output="";
     }
     output+=";";
  }
  stringReplace(output, ',','.');
  if(m_timeCounter >= TIME_WINDOW_OBSERVATION){
     fstrm << output;
  }*/

  /* Contains the velocities that the two wheels will have during the next step.
   * They are initialized with the velocities in the last step because the
   * behavior class could decide to not modify the array.
   */
  Real wheelVelocity[2] = {0.0f, 0.0f};

  if (m_flockingBehavior && !m_firstStep) {
    m_flockingBehavior->UpdateDisplacements(
        tRABReads, tDifReads, m_lastWheelVel, CCI_Controller::GetId());
  }

  bool stop = false;
  for (std::vector<std::shared_ptr<CBehavior>>::iterator i = m_behavior.begin();
       i != m_behavior.end(); i++) {
    if (!stop) {
      stop = (*i).get()->Action(tProxReads, tRABReads, wheelVelocity,
                                CCI_Controller::GetId());
    }
  }

  AvoidCollisions(tProxReads, wheelVelocity);

  if (check_fault(id_int, CConfiguration::FAULT_LACT)) {
    wheelVelocity[0] = 0.0f;
  } else if (check_fault(id_int, CConfiguration::FAULT_RACT)) {
    wheelVelocity[1] = 0.0f;
  } else if (check_fault(id_int, CConfiguration::FAULT_BACT)) {
    wheelVelocity[0] = 0.0f;
    wheelVelocity[1] = 0.0f;
  }

  m_pcWheels->SetLinearVelocity(wheelVelocity[0], wheelVelocity[1]);

  /* Send the message with RAB sensors. It will contain, in order, the id of
   * the sender, the velocity of its left wheel and the velocity of its
   * right wheel during the next step, the beacon byte and the boolean
   * observations of the last control step, if they are used.
   */
  CByteArray cBuf;
  cBuf << id_int;
  cBuf << wheelVelocity[0];
  cBuf << wheelVelocity[1];
  UInt8 beaconByte;
  UInt8 padding = 99;
  if (m_beacon) {
    beaconByte = CConfiguration::BEACON_MESSAGE;
  } else {
    beaconByte = CConfiguration::NOT_BEACON_MESSAGE;
  }
  cBuf << beaconByte;

  // Compare the boolean observations.
  if (CConfiguration::BOOLEAN_OBSERVATIONS) {
    unsigned int numObservedRobots = 0;
    for (int i = 0; i < N_ROBOTS; ++i) {
      if (m_booleanObservations[N_ROBOTS][N_OBSERVATIONS - 1].validity) {
        UInt8 observedRobotId = i;
        cBuf << observedRobotId;
        for (int j = 0; j < 6; ++j) {
          UInt8 observation =
              m_booleanObservations[N_ROBOTS][N_OBSERVATIONS - 1].features[j];
          cBuf << observation;
        }
      }
    }
  }
  cBuf.Resize(26 + 7 * N_ROBOTS, padding);

  // Send the votes of the robot and its local coalition lists.
  if (!m_training &&
      m_timeCounter % (COMPARISON_CONTROL_STEPS + OBSERVE_CONTROL_STEPS) >
          OBSERVE_CONTROL_STEPS) {
    /* Send the votes of the robot. Do not send votes if you already have a
     * local coalition on the observed robot.
     */
    for (int i = 0; i < N_ROBOTS; ++i) {
      if (m_faultProbabilityAccumulator[i].numberOfProbabilites > 0 &&
          !m_localCoalitionList[i].validity) {
        std::string id_str = CCI_Controller::GetId();
        id_str.erase(0, 2);
        UInt8 id_int = std::stoi(id_str);
        cBuf << id_int;
        UInt8 observedRobotId = i;
        cBuf << observedRobotId;
        UInt8 vote;
        if (m_faultProbabilityAccumulator[i].faultProbabilityAccumulator /
                m_faultProbabilityAccumulator[i].numberOfProbabilites >
            CConfiguration::PROBABILITY_THRESHOLD) {
          vote = 1;
        } else {
          vote = 0;
        }
        cBuf << vote;
      }
    }
    cBuf.Resize(26 + 7 * N_ROBOTS + 3 * N_ROBOTS, padding);

    // Send the local coalition lists.
    for (int i = 0; i < N_ROBOTS; ++i) {
      if (m_localCoalitionList[i].validity) {
        UInt8 observedRobotId = i;
        cBuf << observedRobotId;
        UInt8 numberOfVotes = m_localCoalitionList[i].numberOfVotes;
        cBuf << numberOfVotes;
        UInt8 faulty = m_localCoalitionList[i].faulty;
        cBuf << faulty;
      }
    }
  }
  cBuf.Resize(26 + 7 * N_ROBOTS + 3 * N_ROBOTS + 3 * N_ROBOTS, padding);

  m_pcRABA->SetData(cBuf);

  /* Update the array that contains the speed of the wheels in the last
   * step with the speed measured in this step. This part is necessary
   * to have the correct values in the next control step.
   */
  m_lastWheelVel[0] = tDifReads.VelocityLeftWheel;
  m_lastWheelVel[1] = tDifReads.VelocityRightWheel;

  /* Save the highest value of the proximity sensors. */
  m_nearestProx = 0;
  for (size_t i = 0; i < tProxReads.size(); ++i) {
    if (m_nearestProx < tProxReads[i].Value) {
      m_nearestProx = tProxReads[i].Value;
    }
  }

  /* Increase the number of control steps. */
  m_timeCounter++;
}

void CEPuckFaultDetection::GetAndCorrectProxReadings(
    CCI_EPuckProximitySensor::TReadings& correctedReadings) {
  std::string id_str = CCI_Controller::GetId();
  id_str.erase(0, 2);
  UInt8 id_int = std::stoi(id_str);

  const CCI_EPuckProximitySensor::TReadings& sensorsReadings =
      m_pcProximity->GetReadings();

  if (check_fault(id_int, CConfiguration::FAULT_PMIN)) {
    for (int i = 0; i < sensorsReadings.size(); ++i) {
      correctedReadings.push_back(CCI_EPuckProximitySensor::SReading());
      correctedReadings[i].Angle = sensorsReadings[i].Angle;
      // Modify only the frontal readings.
      if (i < sensorsReadings.size() / 4 ||
          i >= sensorsReadings.size() * 3 / 4) {
        correctedReadings[i].Value = 0.0f;
      } else {
        correctedReadings[i].Value = sensorsReadings[i].Value;
      }
    }
  } else if (check_fault(id_int, CConfiguration::FAULT_PMAX)) {
    for (int i = 0; i < sensorsReadings.size(); ++i) {
      correctedReadings.push_back(CCI_EPuckProximitySensor::SReading());
      correctedReadings[i].Angle = sensorsReadings[i].Angle;
      // Modify only the frontal readings.
      if (i < sensorsReadings.size() / 4 ||
          i >= sensorsReadings.size() * 3 / 4) {
        correctedReadings[i].Value = 1.0f;
      } else {
        correctedReadings[i].Value = sensorsReadings[i].Value;
      }
    }
  } else if (check_fault(id_int, CConfiguration::FAULT_PRND)) {
    CRange<Real> range(0.0f, 1.0f);

    for (int i = 0; i < sensorsReadings.size(); ++i) {
      correctedReadings.push_back(CCI_EPuckProximitySensor::SReading());
      correctedReadings[i].Angle = sensorsReadings[i].Angle;
      // Modify only the frontal readings.
      if (i < sensorsReadings.size() / 4 ||
          i >= sensorsReadings.size() * 3 / 4) {
        correctedReadings[i].Value =
            CSourceOfRandomness::m_pcRNG->Uniform(range);
      } else {
        correctedReadings[i].Value = sensorsReadings[i].Value;
      }
    }
  } else {
    correctedReadings = sensorsReadings;
  }
}

void CEPuckFaultDetection::GetAndCorrectRABReadings(
    CCI_RangeAndBearingSensor::TReadings& correctedReadings) {
  /* Detect the id of the robot. */
  std::string id_str = CCI_Controller::GetId();
  id_str.erase(0, 2);
  UInt8 id_int = std::stoi(id_str);

  const CCI_RangeAndBearingSensor::TReadings& sensorsReadings =
      m_pcRABS->GetReadings();

  if (check_fault(id_int, CConfiguration::FAULT_ROFS)) {
    CRange<Real> poolRange(75.0f, 100.0f);
    CRange<CRadians> poolBearing(-CRadians::PI, CRadians::PI);

    for (int i = 0; i < sensorsReadings.size(); ++i) {
      correctedReadings.push_back(CCI_RangeAndBearingSensor::SPacket());
      correctedReadings[i].Data = sensorsReadings[i].Data;
      correctedReadings[i].Range =
          sensorsReadings[i].Range +
          CSourceOfRandomness::m_pcRNG->Uniform(poolRange);
      correctedReadings[i].HorizontalBearing =
          sensorsReadings[i].HorizontalBearing +
          CSourceOfRandomness::m_pcRNG->Uniform(poolBearing);
      correctedReadings[i].VerticalBearing.SetValue(
          sensorsReadings[i].VerticalBearing.GetValue());
    }
  } else {
    correctedReadings = sensorsReadings;
  }
}

void CEPuckFaultDetection::AvoidCollisions(
    CCI_EPuckProximitySensor::TReadings& tProxReads, Real* wheelVelocity) {
  if (tProxReads[0].Value > 0.4f || tProxReads[1].Value > 0.4f ||
      tProxReads[2].Value > 0.4f || tProxReads[3].Value > 0.4f ||
      tProxReads[4].Value > 0.4f || tProxReads[5].Value > 0.4f ||
      tProxReads[6].Value > 0.4f || tProxReads[7].Value > 0.4f) {
    m_consecutive_collisions++;
  } else {
    m_consecutive_collisions = 0u;
  }

  /* 1.0f/TIME_STEP represents the number of iterations of the control step in
   * one second.
   */
  if ((Real)m_consecutive_collisions > 1.0f / (CConfiguration::TIME_STEP)) {
    wheelVelocity[0] = wheelVelocity[0] / 2.0f;
    wheelVelocity[1] = wheelVelocity[1] / 2.0f;
  }
}

void CEPuckFaultDetection::CompareObservations(
    CCI_RangeAndBearingSensor::TReadings& tRABReads) {
  struct ObservationAccumulator {
    unsigned int featuresAccumulator[6];
    unsigned int numberOfObservations;
    bool ownObservation;
  };

  ObservationAccumulator observationAccumulator[N_ROBOTS];

  // Initialize the accumulators.
  for (int i = 0; i < N_ROBOTS; ++i) {
    if (m_booleanObservations[N_ROBOTS][N_OBSERVATIONS - 1].validity) {
      observationAccumulator[i].ownObservation = true;
      observationAccumulator[i].numberOfObservations = 1;
      for (int j = 0; j < 6; ++j) {
        observationAccumulator[i].featuresAccumulator[j] =
            m_booleanObservations[N_ROBOTS][N_OBSERVATIONS - 1].features[j];
      }
    } else {
      observationAccumulator[i].ownObservation = false;
    }
  }

  /* Increase the accumulators with the messages receive from all the other
   * robots.
   */
  for (size_t i = 0; i < tRABReads.size(); ++i) {
    CByteArray message = CByteArray(tRABReads[i].Data);
    bool stop = false;
    unsigned int index = 26;
    unsigned int id;

    while (!stop && index <= 25 + N_ROBOTS * 7) {
      if (message[index] > N_ROBOTS - 1) {
        stop = true;
      } else {
        id = message[index];
        index++;
      }

      if (!stop && observationAccumulator[id].ownObservation) {
        observationAccumulator[id].numberOfObservations++;
        for (int j = 0; j < 6; ++j) {
          observationAccumulator[id].featuresAccumulator[j] += message[index];
          index++;
        }
      }
    }
  }

  // Compute the new BFV for each observed robot and save them.
  for (int i = 0; i < N_ROBOTS; ++i) {
    if (observationAccumulator[i].ownObservation) {
      for (int j = 0; j < 6; ++j) {
        m_booleanObservations[N_ROBOTS][N_OBSERVATIONS - 1].features[j] =
            (observationAccumulator[i].featuresAccumulator[j] >=
             observationAccumulator[i].numberOfObservations / 2.0f);
      }
    }
  }
}

void CEPuckFaultDetection::ObserveNeighborsBoolean(
    CCI_RangeAndBearingSensor::TReadings& tRABReads,
    const CCI_DifferentialSteeringSensor::SReading& tDifReads,
    CCI_EPuckProximitySensor::TReadings& tProxReads) {
  NeighborAccumulator observedNeighborsAccumulators[N_ROBOTS];
  CVector2 observedDisplacements[N_ROBOTS];
  CVector2 observedDisplacementsBayes[N_ROBOTS];
  unsigned int numberOfFeatures[N_ROBOTS];
  BooleanObservation lastObservations[N_ROBOTS];
  for (int i = 0; i < N_ROBOTS; ++i) {
    numberOfFeatures[i] = 0;
  }
  /* Initialize the array. */
  for (int i = 0; i < N_ROBOTS; ++i) {
    observedDisplacements[i] = (CConfiguration::INVALID_DISPLACEMENT_VECTOR);
  }
  /* Initialize the array. */
  for (int i = 0; i < N_ROBOTS; ++i) {
    observedDisplacementsBayes[i] =
        (CConfiguration::INVALID_DISPLACEMENT_VECTOR);
  }

  if (m_firstStep) {
    m_firstStep = false;
  } else {
    /* FEATURES F1 AND F2 */
    computeNeighborsMatrix(tRABReads, &m_neighborsMatrix[0][0],
                           observedNeighborsAccumulators,
                           TIME_WINDOW_OBSERVATION, m_timeCounter,
                           CCI_Controller::GetId(), m_beacon);
    for (int i = 0; i < N_ROBOTS; ++i) {
      if (observedNeighborsAccumulators[i].validity) {
        if ((observedNeighborsAccumulators[i].neighborInnerRangeAccumulator /
             TIME_WINDOW_OBSERVATION) > 0.5) {
          lastObservations[i].features[0] = true;
        } else {
          lastObservations[i].features[0] = false;
        }

        if ((observedNeighborsAccumulators[i].neighborOuterRangeAccumulator /
             TIME_WINDOW_OBSERVATION) > 0.5) {
          lastObservations[i].features[1] = true;
        } else {
          lastObservations[i].features[1] = false;
        }
        numberOfFeatures[i] += 2;
      }
    }

    /* FEATURE F3 */
    bool collision = false;
    if (CConfiguration::COLLISION_CONSERVATIVE) {
      Real proxThreshold = 1e-40;
      if (m_nearestProx >= proxThreshold) {
        collision = true;
      }
    }
    if (!collision) {
      computeDisplacementsMatrix(tRABReads, tDifReads, m_lastWheelVel,
                                 &m_displacementsMatrix[0][0],
                                 observedDisplacements, TIME_WINDOW_OBSERVATION,
                                 m_timeCounter, CCI_Controller::GetId());
      computeDisplacementsMatrix(tRABReads, tDifReads, m_lastWheelVel,
                                 &m_displacementsMatrixBayes[0][0],
                                 observedDisplacementsBayes, TIME_WINDOW_BAYES,
                                 m_timeCounter, CCI_Controller::GetId());
    } else {
      /* If the robot probabily collided in the last control step, make all
       * the displacement measurements invalid. */
      for (int i = 0; i < N_ROBOTS; ++i) {
        for (int j = 0; j < TIME_WINDOW_OBSERVATION; ++j) {
          m_displacementsMatrix[i][j].validity = false;
        }
      }
      for (int i = 0; i < N_ROBOTS; ++i) {
        for (int j = 0; j < TIME_WINDOW_BAYES; ++j) {
          m_displacementsMatrixBayes[i][j].validity = false;
        }
      }
    }
    for (int i = 0; i < N_ROBOTS; ++i) {
      /* To compute feature F3, it is not necessary to control if the
       * observed robot is a beacon because this control was done above to
       * compute F0 and F1. Thus, the beacon has numberOfFeatures = 0 and
       * all its other features will not be computed.*/
      if (numberOfFeatures[i] == 2 &&
          observedDisplacements[i] !=
              CConfiguration::INVALID_DISPLACEMENT_VECTOR) {
        if (observedDisplacements[i].Length() > 0.15 * TIME_WINDOW_OBSERVATION *
                                                    CConfiguration::TIME_STEP *
                                                    m_fWheelVelocity) {
          lastObservations[i].features[2] = true;
        } else {
          lastObservations[i].features[2] = false;
        }
        numberOfFeatures[i] += 1;
      }
    }
    /* FEATURES F4, F5, AND F6 */
    for (size_t i = 0; i < tRABReads.size(); ++i) {
      UInt8 id;
      float observedVelLeftWheel;
      float observedVelRightWheel;
      UInt8 beaconByte;
      CByteArray message = CByteArray(tRABReads[i].Data);
      message >> id;
      message >> observedVelLeftWheel;
      message >> observedVelRightWheel;
      message >> beaconByte;
      /* The max angular acceleration is represented here in cm/control
       * cycle. It represents the case in which the robot, before the control
       * cycle, is rotating in one sense, then it rotates in the other sense.
       * In both situations the rotation is obtained with one wheel at
       * maximum velocity in one direction and the other wheel at maximum
       * velocity in the opposite direction.
       */
      const float maxAngularAcceleration = 2.0f * (2.0f * m_fWheelVelocity) /
                                           CConfiguration::INTERWHEEL_DISTANCE *
                                           CConfiguration::TIME_STEP;

      if (beaconByte != CConfiguration::BEACON_MESSAGE &&
          observedDisplacementsBayes[id] !=
              CConfiguration::INVALID_DISPLACEMENT_VECTOR &&
          numberOfFeatures[id] == 3 &&
          m_prevAngularSpeed[id] != CConfiguration::INVALID_ANGULAR_SPEED) {
        bool neighborsInRange = false;
        bool m;
        float prevAngularSpeed = m_prevAngularSpeed[id];
        float currentAngularSpeed =
            (-observedVelLeftWheel + observedVelRightWheel) /
            CConfiguration::INTERWHEEL_DISTANCE * CConfiguration::TIME_STEP;
        float normalizedAngularAcceleration =
            fabsf(currentAngularSpeed - prevAngularSpeed) /
            maxAngularAcceleration;
        if (normalizedAngularAcceleration *
                observedDisplacementsBayes[id].Length() /
                (TIME_WINDOW_BAYES * CConfiguration::TIME_STEP *
                 m_fWheelVelocity) >
            0.1) {
          m = true;
        } else {
          m = false;
        }
        for (int j = 0; j < tRABReads.size() && !neighborsInRange; ++j) {
          float observedNeighborDistance;
          if (j != i && tRABReads[j].Data[CConfiguration::BEACON_BYTE_INDEX] ==
                            CConfiguration::NOT_BEACON_MESSAGE) {
            observedNeighborDistance =
                sqrt(pow(tRABReads[i].Range, 2) + pow(tRABReads[j].Range, 2) -
                     2 * tRABReads[i].Range * tRABReads[j].Range *
                         Cos(tRABReads[i].HorizontalBearing -
                             tRABReads[j].HorizontalBearing));
            if (observedNeighborDistance < 30) {
              neighborsInRange = true;
            }
          }
        }
        if (!m_beacon && !neighborsInRange) {
          if (tRABReads[i].Range < 30) {
            neighborsInRange = true;
          }
        }

        /* Increase the parameters of the Beta functions. */
        if (neighborsInRange) {
          m_sensPriorBeta[id]++;
          if (m) {
            m_sensPriorAlpha[id]++;
            m_sensPriorBeta[id] -= 1;
          }
        } else {
          m_noSensPriorBeta[id]++;
          if (m) {
            m_noSensPriorAlpha[id]++;
            m_noSensPriorBeta[id] -= 1;
          }
        }
        m_irrespPriorBeta[id]++;
        if (m) {
          m_irrespPriorAlpha[id]++;
          m_irrespPriorBeta[id] -= 1;
        }
        /* Assign the values of the last 3 features.*/
        if (((float)m_irrespPriorAlpha[id] /
             (m_irrespPriorAlpha[id] + m_irrespPriorBeta[id])) > 0.05) {
          lastObservations[id].features[3] = true;
        } else {
          lastObservations[id].features[3] = false;
        }
        if (((float)m_sensPriorAlpha[id] /
             (m_sensPriorAlpha[id] + m_sensPriorBeta[id])) > 0.05) {
          lastObservations[id].features[4] = true;
        } else {
          lastObservations[id].features[4] = false;
        }
        if (((float)m_noSensPriorAlpha[id] /
             (m_noSensPriorAlpha[id] + m_noSensPriorBeta[id])) > 0.05) {
          lastObservations[id].features[5] = true;
        } else {
          lastObservations[id].features[5] = false;
        }
        numberOfFeatures[id] += 3;
      }
    }
    /* Compute the last three features endogenously. */
    if (CConfiguration::ENDOGENOUS_OBSERVATION && !m_beacon) {
      UInt8 id;
      float observedVelLeftWheel;
      float observedVelRightWheel;
      UInt8 beaconByte;
      bool neighborsInRange = false;
      bool m;
      std::string id_str = CCI_Controller::GetId();
      id_str.erase(0, 2);
      UInt8 id_int = std::stoi(id_str);
      const float maxAngularAcceleration = 2.0f * (2.0f * m_fWheelVelocity) /
                                           CConfiguration::INTERWHEEL_DISTANCE *
                                           CConfiguration::TIME_STEP;
      float prevAngularSpeed = (-m_lastWheelVel[0] + m_lastWheelVel[1]) /
                               CConfiguration::INTERWHEEL_DISTANCE *
                               CConfiguration::TIME_STEP;
      float currentAngularSpeed =
          (-tDifReads.VelocityLeftWheel + tDifReads.VelocityRightWheel) /
          CConfiguration::INTERWHEEL_DISTANCE * CConfiguration::TIME_STEP;
      float normalizedAngularAcceleration =
          fabsf(currentAngularSpeed - prevAngularSpeed) /
          maxAngularAcceleration;
      if (normalizedAngularAcceleration *
              observedDisplacementsBayes[id_int].Length() /
              (TIME_WINDOW_BAYES * CConfiguration::TIME_STEP *
               m_fWheelVelocity) >
          0.1) {
        m = true;
      } else {
        m = false;
      }
      for (int i = 0; i < tRABReads.size() && !neighborsInRange; ++i) {
        CByteArray message = CByteArray(tRABReads[i].Data);
        message >> id;
        message >> observedVelLeftWheel;
        message >> observedVelRightWheel;
        message >> beaconByte;
        if (beaconByte != CConfiguration::BEACON_MESSAGE) {
          if (tRABReads[i].Range < 30) {
            neighborsInRange = true;
          }
        }
      }
      /* Increase the parameters of the Beta functions. */
      if (neighborsInRange) {
        m_sensPriorBeta[id_int]++;
        if (m) {
          m_sensPriorAlpha[id_int]++;
          m_sensPriorBeta[id_int] -= 1;
        }
      } else {
        m_noSensPriorBeta[id_int]++;
        if (m) {
          m_noSensPriorAlpha[id_int]++;
          m_noSensPriorBeta[id_int] -= 1;
        }
      }
      m_irrespPriorBeta[id_int]++;
      if (m) {
        m_irrespPriorAlpha[id_int]++;
        m_irrespPriorBeta[id_int] -= 1;
      }
      /* Assign the values of the last 3 features.*/
      if ((m_irrespPriorAlpha[id_int] /
           (m_irrespPriorAlpha[id_int] + m_irrespPriorBeta[id_int])) > 0.05) {
        lastObservations[id_int].features[3] = true;
      } else {
        lastObservations[id_int].features[3] = false;
      }
      if ((m_sensPriorAlpha[id_int] /
           (m_sensPriorAlpha[id_int] + m_sensPriorBeta[id_int])) > 0.05) {
        lastObservations[id_int].features[4] = true;
      } else {
        lastObservations[id_int].features[4] = false;
      }
      if ((m_noSensPriorAlpha[id_int] /
           (m_noSensPriorAlpha[id_int] + m_noSensPriorBeta[id_int])) > 0.05) {
        lastObservations[id_int].features[5] = true;
      } else {
        lastObservations[id_int].features[5] = false;
      }
      numberOfFeatures[id_int] += 3;
    }

    for (int i = 0; i < N_ROBOTS; ++i) {
      if (numberOfFeatures[i] == 6) {
        lastObservations[i].validity = true;
      } else {
        lastObservations[i].validity = false;
      }
      pushBooleanObservation(m_booleanObservations[i], lastObservations[i]);
    }

    /* Update the array containing the angular speeds. */
    bool observed[N_ROBOTS] = {false};
    for (size_t i = 0; i < tRABReads.size(); ++i) {
      UInt8 id;
      float observedVelLeftWheel;
      float observedVelRightWheel;
      UInt8 beaconByte;
      CByteArray message = CByteArray(tRABReads[i].Data);
      message >> id;
      message >> observedVelLeftWheel;
      message >> observedVelRightWheel;
      message >> beaconByte;
      if (beaconByte != CConfiguration::BEACON_MESSAGE) {
        m_prevAngularSpeed[id] =
            (-observedVelLeftWheel + observedVelRightWheel) /
            CConfiguration::INTERWHEEL_DISTANCE * CConfiguration::TIME_STEP;
        observed[id] = true;
      }
    }
    for (int i = 0; i < N_ROBOTS; ++i) {
      if (!observed[i]) {
        m_prevAngularSpeed[i] = CConfiguration::INVALID_ANGULAR_SPEED;
      }
    }
  }
}

void CEPuckFaultDetection::ObserveNeighborsNumerical(
    CCI_RangeAndBearingSensor::TReadings& tRABReads,
    const CCI_DifferentialSteeringSensor::SReading& tDifReads,
    CCI_EPuckProximitySensor::TReadings& tProxReads) {
  UInt8 observedId;
  float observedVelLeftWheel;
  float observedVelRightWheel;
  float observedDisplacement;
  UInt8 beaconByte;
  CVector2 observedDisplacements[N_ROBOTS];
  /* This array controls if a robot has been observed in this control step.
   * Initalized to all false. */
  bool observedRobot[N_ROBOTS] = {false};

  /* Initialize the array. */
  for (int i = 0; i < N_ROBOTS; ++i) {
    observedDisplacements[i] = (CConfiguration::INVALID_DISPLACEMENT_VECTOR);
  }

  /* During the first step, each robot sends a dummy message to connect to its
   * peers. This message must not be saved by the receivers.
   */
  if (m_firstStep) {
    m_firstStep = false;
  } else {
    bool collision = false;

    if (CConfiguration::COLLISION_CONSERVATIVE) {
      Real proxThreshold = 1e-40;

      if (m_nearestProx >= proxThreshold) {
        collision = true;
      }
    }

    if (!collision) {
      computeDisplacementsMatrix(tRABReads, tDifReads, m_lastWheelVel,
                                 &m_displacementsMatrix[0][0],
                                 observedDisplacements, TIME_WINDOW_OBSERVATION,
                                 m_timeCounter, CCI_Controller::GetId());
    } else {
      /* If the robot probabily collided in the last control step, make all the
       * displacement measurements invalid. */
      for (int i = 0; i < N_ROBOTS; ++i) {
        for (int j = 0; j < TIME_WINDOW_OBSERVATION; ++j) {
          m_displacementsMatrix[i][j].validity = false;
        }
      }
    }

    /* Log all the readings from the RAB.*/
    for (size_t i = 0; i < tRABReads.size(); ++i) {
      CByteArray message = CByteArray(tRABReads[i].Data);
      message >> observedId;
      message >> observedVelLeftWheel;
      message >> observedVelRightWheel;
      message >> beaconByte;

      // Do not observe the behavior of the robot that has the beacon role.
      if (beaconByte != CConfiguration::BEACON_MESSAGE) {
        if (observedDisplacements[observedId] !=
            (CConfiguration::INVALID_DISPLACEMENT_VECTOR)) {
          observedDisplacement = observedDisplacements[observedId].Length();
        } else {
          observedDisplacement = CConfiguration::INVALID_DISPLACEMENT;
        }

        /* Compute the distribution of the neighbors of the observed robot. */
        float observedMinNeighborDistance =
            CConfiguration::INVALID_MIN_NEIGHBOR_DISTANCE;
        float observedSumNeighborDistance = 0.0f;
        float observedAverageNeighborDistance = 0.0f;
        float observedNeighborDistance;
        int observedNumNeighbors = 0;

        for (int j = 0; j < tRABReads.size(); ++j) {
          if (j != i && tRABReads[j].Data[CConfiguration::BEACON_BYTE_INDEX] ==
                            CConfiguration::NOT_BEACON_MESSAGE) {
            observedNeighborDistance =
                sqrt(pow(tRABReads[i].Range, 2) + pow(tRABReads[j].Range, 2) -
                     2 * tRABReads[i].Range * tRABReads[j].Range *
                         Cos(tRABReads[i].HorizontalBearing -
                             tRABReads[j].HorizontalBearing));
            observedSumNeighborDistance += observedNeighborDistance;
            observedNumNeighbors += 1;
            if (observedNeighborDistance < observedMinNeighborDistance) {
              observedMinNeighborDistance = observedNeighborDistance;
            }
          }
        }

        /* The observer robot is a neighbor too. The only exception is when
         * the observer is a beacon, because it is not considered as a
         * neighbor.
         */
        if (!m_beacon) {
          observedSumNeighborDistance += tRABReads[i].Range;
          observedNumNeighbors += 1;
          if (tRABReads[i].Range < observedMinNeighborDistance) {
            observedMinNeighborDistance = tRABReads[i].Range;
          }
        }

        if (observedNumNeighbors != 0) {
          observedAverageNeighborDistance =
              observedSumNeighborDistance / observedNumNeighbors;
        }

        NumericalObservation lastObservation = {observedVelLeftWheel,
                                                observedVelRightWheel,
                                                observedDisplacement,
                                                observedMinNeighborDistance,
                                                observedAverageNeighborDistance,
                                                true};

        pushNumericalObservation(m_numericalObservations[observedId],
                                 lastObservation);

        /* The robot has been observed in this control step. */
        observedRobot[observedId] = true;
      }
    }

    /* If the feature is active in the configuration, the robot observes itself.
     */
    if (CConfiguration::ENDOGENOUS_OBSERVATION && !m_beacon) {
      std::string id_str = CCI_Controller::GetId();
      id_str.erase(0, 2);
      UInt8 id_int = std::stoi(id_str);

      float velLeftWheel = tDifReads.VelocityLeftWheel;
      float velRightWheel = tDifReads.VelocityRightWheel;

      float displacement;

      if (observedDisplacements[id_int] !=
          CConfiguration::INVALID_DISPLACEMENT_VECTOR) {
        displacement = observedDisplacements[id_int].Length();
      } else {
        displacement = CConfiguration::INVALID_DISPLACEMENT;
      }

      float minNeighborDistance = CConfiguration::INVALID_MIN_NEIGHBOR_DISTANCE;
      float avgNeighborDistance = 0.0f;
      unsigned int numNeighbors = 0u;

      for (size_t i = 0; i < tRABReads.size(); ++i) {
        if (tRABReads[i].Range < minNeighborDistance) {
          minNeighborDistance = tRABReads[i].Range;
        }
        avgNeighborDistance += tRABReads[i].Range;
        numNeighbors++;
      }

      if (numNeighbors != 0u) {
        avgNeighborDistance /= numNeighbors;
      }

      NumericalObservation ownObservation = {
          velLeftWheel,        velRightWheel,       displacement,
          minNeighborDistance, avgNeighborDistance, true};

      pushNumericalObservation(m_numericalObservations[id_int], ownObservation);

      observedRobot[id_int] = true;
    }

    /* Add dummy data for the robots that were not observed in this control
     * step. */
    for (int j = 0; j < N_ROBOTS; ++j) {
      if (!observedRobot[j]) {
        /* Add an empty observation for that robot. */
        pushNumericalObservation(m_numericalObservations[j],
                                 {0, 0, 0, 0, 0, false});
      }
    }
  }
}

void CEPuckFaultDetection::UpdateFaultProbabilities() {
  for (int i = 0; i < N_ROBOTS; ++i) {
    bool completeObservation = true;
    std::string observations = "";
    for (int j = 0; j < N_OBSERVATIONS; ++j) {
      if (CConfiguration::BOOLEAN_OBSERVATIONS) {
        if (!m_booleanObservations[i][j].validity) {
          completeObservation = false;
        }
        observations += m_booleanObservations[i][j].toCsvString();
      } else {
        if (!m_numericalObservations[i][j].validity ||
            m_numericalObservations[i][j].displacement ==
                CConfiguration::INVALID_DISPLACEMENT ||
            m_numericalObservations[i][j].minNeighborDistance ==
                CConfiguration::INVALID_MIN_NEIGHBOR_DISTANCE) {
          completeObservation = false;
        }
        observations += m_numericalObservations[i][j].toCsvString();
      }
    }
    stringReplace(observations, ',', '.');

    if (completeObservation) {
      float probability;
      observations = "";

      /* Normalization */
      float inputMatrixNum[5 * N_OBSERVATIONS];
      float inputMatrixBin[6 * N_OBSERVATIONS];
      if (!CConfiguration::BOOLEAN_OBSERVATIONS) {
        for (int j = 0; j < N_OBSERVATIONS; ++j) {
          if (m_behavior_str.compare(CConfiguration::AGGREGATE) == 0) {
            inputMatrixNum[j * 5 + 0] =
                (m_numericalObservations[i][j].velLeftWheel - 0.78743926) /
                4.09533154;
            inputMatrixNum[j * 5 + 1] =
                (m_numericalObservations[i][j].velRightWheel - 0.78743926) /
                4.09533154;
            inputMatrixNum[j * 5 + 2] =
                (m_numericalObservations[i][j].displacement - 5.92557105) /
                8.1276298;
            inputMatrixNum[j * 5 + 3] =
                (m_numericalObservations[i][j].minNeighborDistance -
                 13.38839379) /
                13.57544033;
            inputMatrixNum[j * 5 + 4] =
                (m_numericalObservations[i][j].averageNeighborDistance -
                 22.64306708) /
                15.97433554;
            /*observations = observations + std::to_string(inputMatrixNum
            [j*5+0])+";"; observations = observations +
            std::to_string(inputMatrixNum [j*5+1])+";"; observations =
            observations + std::to_string(inputMatrixNum [j*5+2])+";";
            observations = observations + std::to_string(inputMatrixNum
            [j*5+3])+";"; observations = observations +
            std::to_string(inputMatrixNum [j*5+4])+";";*/
          } else if (m_behavior_str.compare(CConfiguration::DISPERSE) == 0) {
            inputMatrixNum[j * 5 + 0] =
                (m_numericalObservations[i][j].velLeftWheel - 4.25749918) /
                2.10906098;
            inputMatrixNum[j * 5 + 1] =
                (m_numericalObservations[i][j].velRightWheel - 4.25749918) /
                2.10906098;
            inputMatrixNum[j * 5 + 2] =
                (m_numericalObservations[i][j].displacement - 38.29590202) /
                16.90179461;
            inputMatrixNum[j * 5 + 3] =
                (m_numericalObservations[i][j].minNeighborDistance -
                 34.41989348) /
                20.13268943;
            inputMatrixNum[j * 5 + 4] =
                (m_numericalObservations[i][j].averageNeighborDistance -
                 70.61601984) /
                22.21833864;
          } else if (m_behavior_str.compare(CConfiguration::FLOCKING) == 0) {
            inputMatrixNum[j * 5 + 0] =
                (m_numericalObservations[i][j].velLeftWheel - 2.40174834) /
                3.09620344;
            inputMatrixNum[j * 5 + 1] =
                (m_numericalObservations[i][j].velRightWheel - 2.40174834) /
                3.09620344;
            inputMatrixNum[j * 5 + 2] =
                (m_numericalObservations[i][j].displacement - 20.99495627) /
                10.19071104;
            inputMatrixNum[j * 5 + 3] =
                (m_numericalObservations[i][j].minNeighborDistance -
                 14.17819407) /
                13.63555213;
            inputMatrixNum[j * 5 + 4] =
                (m_numericalObservations[i][j].averageNeighborDistance -
                 26.64679617) /
                16.36465572;
          } else if (m_behavior_str.compare(CConfiguration::HOMING) == 0) {
            inputMatrixNum[j * 5 + 0] =
                (m_numericalObservations[i][j].velLeftWheel - 3.09571524) /
                2.97165248;
            inputMatrixNum[j * 5 + 1] =
                (m_numericalObservations[i][j].velRightWheel - 3.09571524) /
                2.97165248;
            inputMatrixNum[j * 5 + 2] =
                (m_numericalObservations[i][j].displacement - 24.48300935) /
                15.68166334;
            inputMatrixNum[j * 5 + 3] =
                (m_numericalObservations[i][j].minNeighborDistance -
                 22.26954653) /
                19.82136902;
            inputMatrixNum[j * 5 + 4] =
                (m_numericalObservations[i][j].averageNeighborDistance -
                 45.91643746) /
                25.53002059;
          } else {
            THROW_ARGOSEXCEPTION("Invalid behavior string.");
          }
        }
      } else {
        for (int j = 0; j < N_OBSERVATIONS; ++j) {
          for (int k = 0; k < 6; ++k) {
            inputMatrixBin[j * 6 + k] = m_booleanObservations[i][j].features[k];
          }
        }
      }

      stringReplace(observations, ',', '.');

      if (use_xg_booster()) {
        DMatrixHandle dmatrix;
        if (CConfiguration::BOOLEAN_OBSERVATIONS) {
          safe_xgboost(XGDMatrixCreateFromMat(
              inputMatrixBin, 1, 6 * N_OBSERVATIONS, 10000.0f, &dmatrix));
        } else {
          safe_xgboost(XGDMatrixCreateFromMat(
              inputMatrixNum, 1, 5 * N_OBSERVATIONS, 10000.0f, &dmatrix));
        }

        const bst_ulong* out_shape;
        bst_ulong out_dim = 0;
        const float* out_result = NULL;

        safe_xgboost(XGBoosterPredictFromDMatrix(
            booster, dmatrix,
            "{\"type\": 0, \"training\": false, \"iteration_begin\": 0,\
               \"iteration_end\": 0,\
               \"strict_shape\": true}",
            &out_shape, &out_dim, &out_result));

        probability = *out_result;
      } else {
        float sum;
        if (CConfiguration::BOOLEAN_OBSERVATIONS) {
          std::vector<float> coef(60, 0);
          float intercept;
          if (m_behavior_str == CConfiguration::AGGREGATE) {
            coef = {-0.5255758,  -0.16779712, 0.2411273,   -0.58071765,
                    -0.52568955, 0.0452808,   -0.15752005, 0.24483697,
                    -0.00489347, -0.06457011, -0.01531281, 0.02350765,
                    -0.37394181, 0.1654909,   0.18572142,  0.0325601,
                    0.04289669,  0.00218899,  0.02109214,  -0.12322903,
                    -0.19767609, -0.09785978, -0.02007878, -0.0186156,
                    0.00525923,  -0.22549801, -0.21469018, -0.08392158,
                    -0.01207189, -0.07273215, -0.00337871, -0.04448924,
                    -0.02780185, -0.20201504, 0.09795624,  -0.11760319,
                    -0.01008855, 0.11074349,  0.21510572,  -0.19385052,
                    0.22693109,  -0.25756173, -0.08720132, 0.04054864,
                    0.21195959,  -0.0546561,  0.15793664,  -0.38704823,
                    -0.22533915, 0.26166264,  -0.11454954, -0.61941977,
                    0.20441395,  -0.61816364, -0.71059491, 0.1572795,
                    -0.1533707,  -1.65196666, 0.1240139,   -1.67170522};
            intercept = 4.55981711;
          } else if (m_behavior_str == CConfiguration::DISPERSE) {
            coef = {4.71229746e-01,  -4.95475861e-01, -1.85710169e+00,
                    -1.67767962e-01, -3.76376399e-01, 3.80163528e-01,
                    1.86926277e-01,  -2.36782829e-01, -1.10059433e+00,
                    8.50634999e-02,  2.34071683e-01,  -3.98700784e-02,
                    -2.99164833e-01, 3.04760386e-02,  -5.65901314e-01,
                    5.36652924e-02,  -1.45597482e-02, 1.67637423e-01,
                    3.55258616e-01,  3.12110550e-01,  3.84585522e-02,
                    -2.10594073e-01, 2.09190248e-01,  -2.01608048e-01,
                    -2.17270403e-01, -4.55830684e-01, 1.29452118e-02,
                    3.39710015e-01,  -4.76830519e-01, -1.02316195e-01,
                    -1.52524831e-01, -5.35802951e-01, -3.59557495e-01,
                    -6.44614634e-04, 1.49619598e-01,  1.31570576e-01,
                    8.41003282e-01,  4.15012814e-01,  -7.07776129e-01,
                    -4.44768768e-02, 8.99032912e-02,  1.09593721e-01,
                    -6.56602544e-01, 5.32415974e-02,  -9.27156023e-01,
                    -1.97374242e-01, -1.31576627e-01, 2.73575932e-01,
                    4.56211161e-02,  7.99847965e-02,  -1.33234593e+00,
                    1.57941407e-01,  1.37262159e-01,  -1.84486264e-02,
                    3.40058913e-01,  -1.28725524e+00, -1.80447662e+00,
                    1.85217360e-01,  -2.16563601e-01, -3.40009818e-01};
            intercept = 5.72664389;
          }
          sum = intercept;
          for (int j = 0; j < 60; ++j) {
            sum += coef[j] * m_booleanObservations[i][j / 6].features[j % 6];
          }
        } else {
          std::vector<float> coef(50, 0);
          float intercept;
          if (m_behavior_str == CConfiguration::DISPERSE) {
            coef = {-1.40132609e-01, -1.22553644e-01, -1.58706589e+00,
                    4.59548158e-02,  9.11113144e-02,  -5.17007456e-02,
                    -3.28386368e-02, -5.10768885e-01, 1.57658262e-01,
                    -4.71079889e-02, -4.11917603e-02, -6.31406701e-02,
                    -4.28566911e-01, -6.31055647e-02, 1.28444550e-01,
                    -7.91498164e-02, -6.21547300e-02, 2.86739128e-01,
                    4.83510701e-02,  -1.43129319e-01, 2.29665982e-02,
                    9.72578312e-03,  4.11113038e-01,  -1.40191426e-01,
                    1.24042160e-03,  -3.27973694e-02, -1.99808280e-03,
                    4.34873595e-01,  1.53431825e-01,  -9.66260049e-02,
                    1.79251450e-02,  -4.84185010e-02, 2.70310343e-01,
                    -1.22253205e-01, 1.60584238e-01,  -3.21468423e-02,
                    -3.89419176e-02, 3.36794571e-01,  1.33590323e-01,
                    -6.13119428e-02, -9.28606276e-02, -7.84526692e-02,
                    -2.31090350e-01, 6.44537888e-02,  -3.97229910e-02,
                    -1.60105561e-01, -1.03711560e-01, -6.32299474e-01,
                    -6.04713626e-02, 2.30977803e-01};
            intercept = -2.05433468;
          } else if (m_behavior_str == CConfiguration::FLOCKING) {
            coef = {
                -0.09897857, -0.08683635, -1.62864043, 0.09771668,  0.02054638,
                -0.0827564,  -0.0635599,  -1.16469369, -0.0153803,  0.02708952,
                -0.0592612,  -0.06968587, -0.46472881, 0.13619741,  -0.01838992,
                -0.10305295, -0.08790605, -0.31681486, -0.14456583, -0.0423143,
                -0.06919569, -0.07225145, 0.00912332,  0.09800351,  0.03974052,
                -0.09186092, -0.10043608, 0.3218838,   0.08866995,  0.04662667,
                -0.09247033, -0.09045124, 0.41117892,  -0.02590526, 0.02671375,
                -0.09264751, -0.06581257, 0.39280796,  0.45024887,  -0.00875648,
                -0.09301125, -0.12119788, 0.6533595,   0.41840311,  0.09222907,
                -0.14584664, -0.14949432, 0.67208438,  0.34113341,  0.12415187};
            intercept = -1.91162457;
          }
          sum = intercept;
          for (int j = 0; j < 50; ++j) {
            sum += coef[j] * inputMatrixNum[j];
          }
        }
        probability = 1 / (1 + exp(-sum));
      }

      /* DUMMY CLASSIFIER */
      /*CRange<Real> range(0.0f, 1.0f);
      if(CSourceOfRandomness::m_pcRNG->Uniform(range)<=0.445){
         probability=1.0f;
      }else{
         probability=0.0f;
      }*/

      if (probability >= 0.0f && probability <= 1.0f) {
        m_faultProbabilityAccumulator[i].faultProbabilityAccumulator +=
            probability;
        m_faultProbabilityAccumulator[i].numberOfProbabilites++;
      } else {
        THROW_ARGOSEXCEPTION(
            "The returned probability is outside range [0,1].");
      }
    }
  }
}

void CEPuckFaultDetection::WriteToCSVTraining() {
  if (CConfiguration::BOOLEAN_OBSERVATIONS) {
    for (int i = 0; i < N_ROBOTS; ++i) {
      bool write = true;

      std::string datapoint =
          std::to_string(CSourceOfRandomness::m_pcRNG->GetSeed()) + ";" +
          std::to_string(m_timeCounter) + ";" + std::to_string(i) + ";";

      if (m_timeCounter < 1000) {
        write = false;
      }
      for (int j = 0; j < N_OBSERVATIONS; ++j) {
        datapoint += m_booleanObservations[i][j].toCsvString();
        if (!m_booleanObservations[i][j].validity) {
          write = false;
        }
      }
      datapoint += std::to_string(m_faultProbabilities[i]);

      stringReplace(datapoint, ',', '.');

      if (write) {
        output_file << datapoint << std::endl;
      }
    }
  } else {
    for (int i = 0; i < N_ROBOTS; ++i) {
      bool write = true;

      std::string datapoint =
          std::to_string(CSourceOfRandomness::m_pcRNG->GetSeed()) + ";" +
          std::to_string(m_timeCounter) + ";" + std::to_string(i) + ";";

      if (m_timeCounter < 1000) {
        write = false;
      }

      for (int j = 0; j < N_OBSERVATIONS; ++j) {
        datapoint += m_numericalObservations[i][j].toCsvString();
        if (!m_numericalObservations[i][j].validity) {
          write = false;
        }
      }
      datapoint += std::to_string(m_faultProbabilities[i]);

      stringReplace(datapoint, ',', '.');

      if (write) {
        output_file << datapoint << std::endl;
      }
    }
  }
}

void CEPuckFaultDetection::WriteToCSVTesting() {
  for (int i = 0; i < N_ROBOTS; ++i) {
    if (m_localCoalitionList[i].validity) {
      output_file << std::to_string(m_timeCounter) + ";" + std::to_string(i) +
                         ";" +
                         std::to_string(m_localCoalitionList[i].numberOfVotes) +
                         ";" + std::to_string(m_localCoalitionList[i].faulty) +
                         ";";
      if (!check_fault(i, CConfiguration::FAULT_NONE)) {
        output_file << "1";
      } else {
        output_file << "0";
      }
      output_file << std::endl;
    }
  }
}

void CEPuckFaultDetection::ReadMessagesCoalitionFormation(
    CCI_RangeAndBearingSensor::TReadings& tRABReads) {
  // Read votes of other robots.
  for (size_t i = 0; i < tRABReads.size(); ++i) {
    CByteArray message = CByteArray(tRABReads[i].Data);
    unsigned int index = 26 + 7 * N_ROBOTS;
    bool stop = false;

    while (!stop && index < 26 + 7 * N_ROBOTS + 3 * N_ROBOTS) {
      if (message[index] > N_ROBOTS) {
        stop = true;
      }
      if (!stop) {
        unsigned int id_observer = message[index];
        index++;
        unsigned int id_observed = message[index];
        index++;
        bool vote = message[index];
        index++;
        /* Store the vote only if the robot is not already in a local
         * coalition list and its vote has not already been registered.
         */
        if (!m_localCoalitionList[id_observed].validity &&
            std::find(m_votesAccumulator[id_observed].idVotersList.begin(),
                      m_votesAccumulator[id_observed].idVotersList.end(),
                      id_observer) ==
                m_votesAccumulator[id_observed].idVotersList.end()) {
          m_votesAccumulator[id_observed].idVotersList.push_back(id_observer);
          if (vote) {
            m_votesAccumulator[id_observed].numFaultyVotes++;
          } else {
            m_votesAccumulator[id_observed].numNormalVotes++;
          }
        }
      }
    }
  }

  // Control if it is possible to create new coalitions.
  for (int i = 0; i < N_ROBOTS; ++i) {
    if (!m_localCoalitionList[i].validity &&
        m_votesAccumulator[i].idVotersList.size() >= 5) {
      m_localCoalitionList[i].validity = true;
      m_localCoalitionList[i].numberOfVotes =
          m_votesAccumulator[i].idVotersList.size();
      if (m_votesAccumulator[i].numFaultyVotes >
          m_votesAccumulator[i].numNormalVotes) {
        m_localCoalitionList[i].faulty = true;
      } else {
        m_localCoalitionList[i].faulty = false;
      }
    }
  }

  // Update the list of coalitions with the ones received in the messages.
  for (size_t i = 0; i < tRABReads.size(); ++i) {
    CByteArray message = CByteArray(tRABReads[i].Data);
    unsigned int index = 26 + 7 * N_ROBOTS + 3 * N_ROBOTS;
    bool stop = false;

    while (!stop && index < 26 + 7 * N_ROBOTS + 3 * N_ROBOTS + 3 * N_ROBOTS) {
      if (message[index] > N_ROBOTS) {
        stop = true;
      }
      if (!stop) {
        unsigned int id_observed = message[index];
        index++;
        unsigned int numberOfVotes = message[index];
        index++;
        bool faulty = message[index];
        index++;
        if (!m_localCoalitionList[id_observed].validity) {
          m_localCoalitionList[id_observed].validity = true;
          m_localCoalitionList[id_observed].numberOfVotes = numberOfVotes;
          m_localCoalitionList[id_observed].faulty = faulty;
        }
      }
    }
  }
}

bool CEPuckFaultDetection::use_xg_booster() const {
  if (CConfiguration::BOOLEAN_OBSERVATIONS) {
    if (m_behavior_str == "floc" || m_behavior_str == "homi") {
      return true;
    }
  } else {
    if (m_behavior_str == "aggr" || m_behavior_str == "homi") {
      return true;
    }
  }
  return false;
}

bool CEPuckFaultDetection::check_fault(int id, std::string fault) const {
  std::string real_fault;
  if (id == CConfiguration::ID_FAULTY_ROBOT) {
    real_fault = m_faultType;
  } else {
    real_fault = CConfiguration::FAULT_NONE;
  }
  return (fault.compare(real_fault) == 0);
}

std::string CEPuckFaultDetection::generate_output_filename() const {
  int random_seed = CSimulator::GetInstance().GetRandomSeed();
  std::string filename = "";
  filename = "./data/binary_training/" + m_behavior_str + "/" + m_behavior_str +
             "_" + m_faultType + "_" + std::to_string(random_seed) + ".csv";
  std::string testing_or_training = "testing";
  std::string binary_or_numerical = "numerical";
  std::string behav_fault_folder = m_behavior_str + "_" + m_faultType + "/";
  if (m_training) {
    testing_or_training = "training";
    behav_fault_folder = "";
  }
  if (CConfiguration::BOOLEAN_OBSERVATIONS) {
    binary_or_numerical = "binary";
  }
  filename = "./data/" + binary_or_numerical + "_" + testing_or_training + "/" +
             m_behavior_str + "/" + behav_fault_folder + m_behavior_str + "_" +
             m_faultType + "_" + std::to_string(random_seed) + ".csv";

  return filename;
}

/****************************************/
/****************************************/

/*
 * This statement notifies ARGoS of the existence of the controller.
 * It binds the class passed as first argument to the string passed as
 * second argument.
 * The string is then usable in the configuration file to refer to this
 * controller.
 * When ARGoS reads that string in the configuration file, it knows which
 * controller class to instantiate.
 * See also the configuration files for an example of how this is used.
 */
REGISTER_CONTROLLER(CEPuckFaultDetection, "epuck_fault_detection")
