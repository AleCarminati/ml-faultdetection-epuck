#ifndef FAULT_DETECTION_H
#define FAULT_DETECTION_H

/*
 * Include some necessary headers.
 */
/* 2D vector definition */
#include <argos3/core/utility/math/vector2.h>
/* Definition of the CCI_Controller class. */
#include <argos3/core/control_interface/ci_controller.h>
/* Definition of the differential steering actuator */
#include <argos3/plugins/robots/generic/control_interface/ci_differential_steering_actuator.h>
/* Definition of the differential steering sensor */
#include <argos3/plugins/robots/generic/control_interface/ci_differential_steering_sensor.h>
/* Definition of proximity sensor */
#include <argos3/plugins/robots/e-puck/control_interface/ci_epuck_proximity_sensor.h>
/* Definition of the range and bearing actuator */
#include <argos3/plugins/robots/generic/control_interface/ci_range_and_bearing_actuator.h>
/* Definition of the range and bearing sensor */
#include <argos3/plugins/robots/generic/control_interface/ci_range_and_bearing_sensor.h>
/* Used only to get the random seed of the experiment. */
#include <argos3/core/simulator/simulator.h>
/* Definitions of smart pointers. */
#include <memory>
/* Mathematical operations. */
#include <math.h>
/* Write on file. */
#include <fstream>
/* Util functions. */
#include "../util/util.h"
/* Class to compute the behavior of the robot. */
#include "../behaviors/aggregatebehavior.h"
#include "../behaviors/behavior.h"
#include "../behaviors/dispersebehavior.h"
#include "../behaviors/flockingbehavior.h"
#include "../behaviors/homingbehavior.h"
#include "../behaviors/randomwalkbehavior.h"

/* Definition of the positioning sensor */
#include <argos3/plugins/robots/generic/control_interface/ci_positioning_sensor.h>

#define PY_SSIZE_T_CLEAN
#include <Python.h>
#include <xgboost/c_api.h>

/*
 * All the ARGoS stuff in the 'argos' namespace.
 * With this statement, you save typing argos:: every time.
 */
using namespace argos;

/*
 * A controller is simply an implementation of the CCI_Controller class.
 */
class CEPuckFaultDetection : public CCI_Controller {
 public:
  /* Class constructor. */
  CEPuckFaultDetection();

  /* Class destructor. */
  virtual ~CEPuckFaultDetection() {}

  /*
   * This function initializes the controller.
   * The 't_node' variable points to the <parameters> section in the XML
   * file in the <controllers><footbot_diffusion_controller> section.
   */
  virtual void Init(TConfigurationNode& t_node);

  /*
   * This function is called once every time step.
   * The length of the time step is set in the XML file.
   */
  virtual void ControlStep();

  /*
   * This function resets the controller to its state right after the
   * Init().
   * It is called when you press the reset button in the GUI.
   * In this example controller there is no need for resetting anything,
   * so the function could have been omitted. It's here just for
   * completeness.
   */
  virtual void Reset() {}

  /*
   * Called to cleanup what done by Init() when the experiment finishes.
   * In this example controller there is no need for clean anything up,
   * so the function could have been omitted. It's here just for
   * completeness.
   */
  virtual void Destroy() {}

  /*
   * This function gets the proximity sensors' readings.
   * It corrects them if the robot is faulty.
   */
  void GetAndCorrectProxReadings(
      CCI_EPuckProximitySensor::TReadings& sensorsReadings);

  /*
   * This function gets the RAB's readings.
   * It corrects them if the robot is faulty.
   */
  void GetAndCorrectRABReadings(
      CCI_RangeAndBearingSensor::TReadings& sensorsReadings);

  /* This function changes the speed of the wheels if the robot collides with
   * something for more than one second. Value taken from
   * Tarapore_PLOSONE_2017, more precisely from epuck_hom_swarm.cpp.
   */
  void AvoidCollisions(CCI_EPuckProximitySensor::TReadings& tProxReads,
                       Real* wheelVelocity);

  /* This function reads the messages of the other robots to get their
   * observations and compare them with its own. Used only with boolean
   * features.
   */
  void CompareObservations(CCI_RangeAndBearingSensor::TReadings& tRABReads);

  /* This function computes the boolean observations of the robots on their
   * neighbors, based on sensor readings.
   */
  void ObserveNeighborsBoolean(
      CCI_RangeAndBearingSensor::TReadings& tRABReads,
      const CCI_DifferentialSteeringSensor::SReading& tDifReads,
      CCI_EPuckProximitySensor::TReadings& tProxReads);

  /* This functions computes the numerical observations of the robots on their
   * neighbors, based on sensor readings.
   */
  void ObserveNeighborsNumerical(
      CCI_RangeAndBearingSensor::TReadings& tRABReads,
      const CCI_DifferentialSteeringSensor::SReading& tDifReads,
      CCI_EPuckProximitySensor::TReadings& tProxReads);

  /* This function updates the fault probabilities of all the robots in the
   * swarm. */
  void UpdateFaultProbabilities();

  /* This function writes the observations and the fault probabilities in a CSV
   *  file during the training phase. */
  void WriteToCSVTraining();

  /* This function writes the results of the local coalitions in a CSV file
   * during testing.
   */
  void WriteToCSVTesting();

  /* This function, during the coalition formation phase, reads the messages
   * from the other robots and updates the data structures related to this
   * phase.
   */
  void ReadMessagesCoalitionFormation(
      CCI_RangeAndBearingSensor::TReadings& tRABReads);

 private:
  /* Pointer to the differential steering actuator */
  CCI_DifferentialSteeringActuator* m_pcWheels;
  /* Pointer to the differential steering sensor */
  CCI_DifferentialSteeringSensor* m_pcWheelsSens;
  /* Pointer to the foot-bot proximity sensor */
  CCI_EPuckProximitySensor* m_pcProximity;
  /* Pointer to the range and bearing actuator */
  CCI_RangeAndBearingActuator* m_pcRABA;
  /* Pointer to the range and bearing sensor */
  CCI_RangeAndBearingSensor* m_pcRABS;
  /* Pointer to the positioning sensor */
  CCI_PositioningSensor* m_pcPosSens;

  /* This vector is used ONLY for testing purposes. It does not affect the fault
   * detection algorithm.
   */
  std::vector<CCI_PositioningSensor::SReading> m_lastPosVector;

  /* Wheel speed. */
  Real m_fWheelVelocity;

  /* Contains the string that defines the type of behavior that the swarm
   * follows. */
  std::string m_behavior_str;

  /* Vector that contains the classes that compute the behavior of the robot.
   * shared_ptr gives the possibility to maintain the object outside the Init
   * function and to use polymorphism. Normal pointers would have lost the
   * object outside the Init function, causing segmentation fault. We use the
   * "shared" version because in the case of flocking behavior there will be
   * another variable that points to that behavior and that is used to update
   * the displacements.
   */
  std::vector<std::shared_ptr<CBehavior>> m_behavior;

  /* This pointer is used when the flocking behavior is selected and refers
   * to a flocking behavior instance. It is used to update the displacements.
   * If the flocking behavior is not selected, this pointer will be NULL, thus
   * it is necessary to check if it is NULL every time it is used, otherwise
   * a NullPointerException will be thrown.
   */
  std::shared_ptr<CFlockingBehavior> m_flockingBehavior;

  /* Data structures that contain the last observations of the
   * robots, the fault  probabilities based on that observations and the last
   * reading of the RAB for each other robot.
   * Only one of the two observation matrices are used, based on what type
   * of observations (boolean or numerical) the experiment is using.
   */
  NumericalObservation m_numericalObservations[N_ROBOTS][N_OBSERVATIONS];
  BooleanObservation m_booleanObservations[N_ROBOTS][N_OBSERVATIONS];
  float m_faultProbabilities[N_ROBOTS];

  /* It contains the data necessary to compute the number of control steps
   * with at least one neighbor in the inner range and in the outer range.
   */
  NeighborAccumulator m_neighborsMatrix[N_ROBOTS][TIME_WINDOW_OBSERVATION];

  /* It contains the data necessary to compute the displacement of the robots
   * in the last TIME_WINDOW_OBSERVATION steps.
   */
  DisplacementAccumulator m_displacementsMatrix[N_ROBOTS]
                                               [TIME_WINDOW_OBSERVATION];

  /* Defines if the robot is in his first control step or not. */
  bool m_firstStep;

  /* Contains the wheels' speed measured in the last control step.
   * It is useful to compute the distance covered by the other robots.
   * In the first position, there will be the speed of the left wheel,
   * in the second position, there will be the speed of the right wheel.
   */
  float m_lastWheelVel[2];

  /* Contains the type of fault that affects the swarm.*/
  std::string m_faultType;

  /* Defines if the experiment is run to get data for training. */
  bool m_training;

  /* Defines the number of consecutive control steps in which the robot is
   * colliding. */
  unsigned int m_consecutive_collisions;

  /* Defines if the robot is a beacon. It is true only when the selected
   * behavior is homing and the robot has the ID equal to BEACON_ID.
   */
  bool m_beacon;

  /* Maintains the highest proximity sensor's value in the last control step.
   * It is used to check if the robot could have collided during the last
   * control step.
   */
  Real m_nearestProx;

  /* It counts the number of control steps from the start of the simulations. */
  unsigned int m_timeCounter;

  /* These variables represents the two parameters (alpha and beta) of the Beta
   * probability distributions that represent the probability of theta given
   * the number of times the observed robot changed its heading:
   * - in any case;
   * - when there aren't neighbors nearby
   * - when there are neighobrs nearby
   * where theta is the fraction of control cycles in which the robot alters
   * its heading.
   * They are used only when boolean features are computed.
   * For more informations, refer to Tarapore_PLOSONE_2017.
   */
  unsigned int m_irrespPriorAlpha[N_ROBOTS];
  unsigned int m_irrespPriorBeta[N_ROBOTS];
  unsigned int m_noSensPriorAlpha[N_ROBOTS];
  unsigned int m_noSensPriorBeta[N_ROBOTS];
  unsigned int m_sensPriorAlpha[N_ROBOTS];
  unsigned int m_sensPriorBeta[N_ROBOTS];

  /* It contains the data necessary to compute the displacement of the robots
   * in the last TIME_WINDOW_BAYES steps. It is used only to compute the
   * last three boolean features.
   */
  DisplacementAccumulator m_displacementsMatrixBayes[N_ROBOTS]
                                                    [TIME_WINDOW_BAYES];

  /* It contains the angular speeds of the other robots in the last control
   * step. Used only to compute the last three boolean features.
   * The cell related to the robot itself is not going to be used because
   * that data can be computed using m_lastWheelVel.
   */
  float m_prevAngularSpeed[N_ROBOTS];

  /* It accumulates the fault probabilities for each robot, computed during
   * the observation control steps.
   */
  FaultProbabilityAccumulator m_faultProbabilityAccumulator[N_ROBOTS];

  /* It contains the local coalitions that are generated during the result
   * comparison phase.
   */
  LocalCoalition m_localCoalitionList[N_ROBOTS];

  /* It contains the votes of the robot itself and of the other robots,
   * obtained during the result comparison phase.
   */
  VotesAccumulator m_votesAccumulator[N_ROBOTS];

  BoosterHandle booster;

  /* Filestream that connects to the output file.
   */
  std::ofstream output_file;

  /* This function returns if the classifier that the robot must use is
   * an extreme gradient booster.
   */
  bool use_xg_booster();
};

#endif
