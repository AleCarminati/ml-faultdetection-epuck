/* ---------------------------------------------------------------- */
/* 								UTIL FILE
 * This file groups all the auxiliary functions, definitions and data
 * structures that are used in different files of this project.
 */
/* ---------------------------------------------------------------- */

#ifndef UTIL_H_
#define UTIL_H_

#include <argos3/core/utility/configuration/argos_exception.h>
#include <argos3/core/utility/math/angles.h>
#include <argos3/core/utility/math/rng.h>
#include <argos3/core/utility/math/vector2.h>
#include <argos3/plugins/robots/generic/control_interface/ci_differential_steering_sensor.h>
#include <argos3/plugins/robots/generic/control_interface/ci_range_and_bearing_sensor.h>
#include <math.h>

#include <algorithm>
#include <fstream>
#include <iostream>
#include <map>
#include <sstream>
#include <string>

/* The number of observations related to a specific node that a robot keeps in
 * memory. */
#define N_OBSERVATIONS 10
/* The number of robots in the swarm. */
#define N_ROBOTS 20
/* The number of control steps necessary to measure the displacement of an
 * observed robot. */
#define TIME_WINDOW_OBSERVATION 100
/* The number of control steps necessary to measure the displacement of an
 * observed robot required to compute the boolean feature values.
 */
#define TIME_WINDOW_BAYES 50
// The number of control steps used to observe and classify the neighbors.
#define OBSERVE_CONTROL_STEPS 90
/* The number of control steps used to confront the classification results via
 * coalition formation.
 */
#define COMPARISON_CONTROL_STEPS 10

using namespace argos;

/* This class reads from a configuration file all the constants useful to run
 * the program. */
class CConfiguration {
 public:
  // This variable defines if the configuration file has already been read.
  static bool initialized;

  /* GENERAL */
  /* The distance between the wheels of the robot. Represented in cm. The value
   * is taken from Tarapore_PLOSONE_2017.
   */
  static float INTERWHEEL_DISTANCE;
  /* Half of the interwheel distance of the robot. Represented in cm. */
  static float HALF_INTERWHEEL_DISTANCE;
  /* The maximum force that can be applied to the robot to change its speed.
   * Represented in Newton. */
  static float MAX_FORCE;
  /* The mass of the robot. Represented in kg. */
  static float MASS;
  /* The acceleration that the robot applies when it needs to change its speed.
   * Represented in cm/s². */
  static float ACCELERATION;
  /* The time elapsed between two different control steps. Represented in s. */
  static float TIME_STEP;
  /* The time elapsed between two different time steps of the physical engine.
   * Represented in s. */
  static float TIME_STEP_PHYSICS;

  /* FAULTS */
  /* The strings that define the different faults. */
  static std::string FAULT_NONE;
  static std::string FAULT_PMIN;
  static std::string FAULT_PMAX;
  static std::string FAULT_PRND;
  static std::string FAULT_ROFS;
  static std::string FAULT_LACT;
  static std::string FAULT_RACT;
  static std::string FAULT_BACT;
  /* The id of the faulty robot. */
  static unsigned int ID_FAULTY_ROBOT;

  /* OBSERVATION */
  /* The value given to the observed displacement when it was not possible to
   * compute it. */
  static int INVALID_DISPLACEMENT;
  /* The value given to the minimum distance of the neighbor when no neighbors
   * are seen. It is higher than any possible value that the variable can
   * assume. */
  static float INVALID_MIN_NEIGHBOR_DISTANCE;
  /* The value given to the fault probability when it was not possible to
   * compute it. */
  static int INVALID_FAULT_PROBABILITY;
  /* The value given to the angular speed of another node when it was not
   * possible to compute it. */
  static float INVALID_ANGULAR_SPEED;
  /* Represents if the conservative solution of the collision problem must be
   * used (true value) or not (false value). The conservative solution of the
   * collision problem makes all the observations of a robot invalid if that
   * robot probably collided in the last control step.
   */
  static bool COLLISION_CONSERVATIVE;
  /* Represents if the robot must observe also its own behavior. */
  static bool ENDOGENOUS_OBSERVATION;
  /* Represents if the observed features are boolean (true) or numerical
   * (false).
   */
  static bool BOOLEAN_OBSERVATIONS;
  /* Represents the threshold of the fault probability used to discriminate
   * between a faulty robot and a non-faulty one.
   */
  static float PROBABILITY_THRESHOLD;

  /* BEHAVIOR */
  /* The strings that represent the different behaviors of the robot. */
  static std::string AGGREGATE;
  static std::string DISPERSE;
  static std::string FLOCKING;
  static std::string HOMING;
  /* The value of the thresholds that define when the robot must go straight,
   * make a soft turn or an hard turn. They are taken from the code of
   * Tarapore_PLOSONE_2017, more precisely from the file epuck_hom_swarm.h.
   */
  static CRadians NO_TURN_THRESHOLD;
  static CRadians SOFT_TURN_THRESHOLD;
  /* The value of the integers that represents the type of turn that the robot
   * must do. */
  static int NO_TURN;
  static int SOFT_TURN;
  static int HARD_TURN;
  /* The threshold beyond which the RAB readings are not considered
   * for the behavior. This value is taken from Tarapore_PLOSONE_2017, more
   * precisely from epuck_hom_swarm.cpp.
   */
  static float RANGE_THRESHOLD_AGGREGATION;
  /* A dummy vector that is used when it was not possible to compute the
   * displacement. */
  static CVector2 INVALID_DISPLACEMENT_VECTOR;
  /* Defines the threshold of the proximity sensor beyond which turn the robot.
   * This value is taken from Tarapore_PLOSONE_2017, more precisely from
   * epuck_hom_swarm.cpp.
   */
  static float PROXIMITY_SENSOR_THRESHOLD_DISPERSION;
  /* Defines the threshold of the angle obtained by the proximity sensors beyond
   * which turn the robot. This value is taken from Tarapore_PLOSONE_2017, more
   * precisely from epuck_hom_swarm.cpp.
   */
  static CRadians GO_STRAIGHT_ANGLE_THRESHOLD_DISPERSION;
  /* Defines the probability for the robot to change its direction.
   * This value is taken from Tarapore_PLOSONE_2017, more precisely from
   * epuck_hom_swarm.cpp.
   */
  static float CHANGE_DIRECTION_PROBABILITY_RANDOMWALK;
  /* The bytes to differentiate the messages that come from a beacon and the
   * ones that come from a normal robot. */
  static unsigned int NOT_BEACON_MESSAGE;
  static unsigned int BEACON_MESSAGE;
  /* The ID of the robot that must be the beacon in the homing behavior. */
  static unsigned int BEACON_ID;
  /* The index of the byte that identifies it the sender is a beacon or not. It
   * is computed with the assumption that the first position has index equal to
   * 0.
   */
  static unsigned int BEACON_BYTE_INDEX;

  /* MULTIFAULT */
  /* The probability that a single robot is faulty. */
  static float PROB_FAULTY;

  /* It contains the informations about faulty robots. Each key is the id of a
   * faulty robot and it is linked to its fault.
   */
  static std::map<unsigned, std::string> FAULTY_ROBOTS_MAP;

  /* This function reads the configuration file and initializes all the
   * variables of the class. It contains a simple parser to read the .ini files,
   * which supports sections and single-line comments that start with '#'.
   */
  static void ReadConfigurationFile();

  /* This function prints all the variables in configuration and their
   * respective values. It can be used to check if the configuration file has
   * been read correctly.
   */
  static void PrintConfiguration();

 private:
  // This variable contains the relative path to reach the configuration file.
  const static std::string configurationFilePath;
};

/* This class encapsulates a source of randomness that is declared as a static
 * member to have a unique source of randomness across all the files.
 * Thus, each experiment is repeatable because it is associated with a seed.
 * The value of the seed must be written in the config file.
 */
class CSourceOfRandomness {
 public:
  static CRandom::CRNG* m_pcRNG;
};

/*
 * NumericalObservation contains the data of a numerical observation on a
 * particular robot.
 */
struct NumericalObservation {
  float velLeftWheel;
  float velRightWheel;
  float displacement;
  float minNeighborDistance;
  float averageNeighborDistance;
  bool validity;

  /*
   * This method transforms the data structure in a string that can be inserted
   * into a .csv file.
   */
  std::string toCsvString() {
    std::string csvString;
    if (validity) {
      csvString = std::to_string(velLeftWheel) + ";" +
                  std::to_string(velRightWheel) + ";";
      /* In order to compute the distance traveled, two consecutive observations
       * are needed. The first of the two observations will have the value of
       * the distance traveled equal to a dummy value, because it was impossible
       * to compute it.
       */
      if (displacement != CConfiguration::INVALID_DISPLACEMENT) {
        csvString += std::to_string(displacement);
      }
      csvString += ";";

      if (minNeighborDistance !=
          CConfiguration::INVALID_MIN_NEIGHBOR_DISTANCE) {
        csvString += std::to_string(minNeighborDistance) + ";" +
                     std::to_string(averageNeighborDistance) + ";";
      } else {
        csvString += ";;";
      }

    } else {
      csvString = ";;;;;";
    }
    return csvString;
  }
};

/*
 * BooleanObservation contains the data of a numerical observation on a
 * particular robot.
 */
struct BooleanObservation {
  bool features[6];
  bool validity;

  /*
   * This method transforms the data structure in a string that can be inserted
   * into a .csv file.
   */
  std::string toCsvString() {
    std::string csvString = "";
    if (validity) {
      for (int i = 0; i < 6; ++i) {
        csvString += std::to_string(features[i]) + ";";
      }
    } else {
      csvString = ";;;;;;";
    }
    return csvString;
  }
};

/* Auxiliary structure that, given time t, stores the observed robot's position
 * at time t, the future movements of the observer robot w.r.t. the observer
 * robot's reference frame at time t. Its data will be used at time t +
 * time_window to compute the displacement of the observed robot during
 * time_window. Time t and the id of the observed robot are not explicitly
 * represented in this structure because their management is left to the class
 * that uses the structure.
 */
struct DisplacementAccumulator {
  /* The initial position of the observed robot. */
  CVector2 initialPosition;

  /* Two accumulators. They store, respectively, the translation and the
   * rotation of the observer from time t to the current control step, both
   * w.r.t. the observer robot's reference frame at time t.
   */
  CVector2 observerTranslation;
  CRadians observerRotation;

  /* This variable defines if the accumulator is valid and can be used to
   * compute a displacement. It will be true if the initial position of the
   * observed robot at time t was observed, otherwise it will be false.
   */
  bool validity;
};

/* Auxiliary structure that accumulates the number of control steps when there's
 * at least one neighbor in the inner range (0-15cm) and in the outer range
 * (15-30cm) of a robot.
 * Used to compute the boolean features F1 and F2 described in
 * Tarapore_PLOSONE_2017.
 */
struct NeighborAccumulator {
  unsigned int neighborInnerRangeAccumulator;
  unsigned int neighborOuterRangeAccumulator;
  bool validity;
};

/* This structure accumulates the fault probabilities about one robot during the
 * observation control steps.
 */
struct FaultProbabilityAccumulator {
  float faultProbabilityAccumulator;
  unsigned int numberOfProbabilites;
};

/* This structure contains the informations about a local coalition. It is used
 * in the coalition formation phase.
 */
struct LocalCoalition {
  unsigned int numberOfVotes;
  bool faulty;
  bool validity;
};

/* This structure accumulates the votes on a particular robot, when there are
 * not enough votes to form a coalition.
 */
struct VotesAccumulator {
  std::vector<unsigned int> idVotersList;
  unsigned int numNormalVotes;
  unsigned int numFaultyVotes;
};

/* This function computes the neighbors accumulator from a matrix of
 * NeighborsAccumulator and updates the matrix.
 * The matrix must have N_ROBOTS rows, one for each robot in the swarm, and a
 * number of columns equal to the time window used to compute the displacement.
 * The results of the displacement computations will be inserted in an array of
 * length equal to the number of robots in the swarm. The measure of the time
 * must be provided by the caller function.
 */
void computeNeighborsMatrix(CCI_RangeAndBearingSensor::TReadings& tRABReads,
                            NeighborAccumulator* matrix,
                            NeighborAccumulator* accumulators,
                            unsigned int TIME_WINDOW, unsigned int time,
                            std::string id_str, bool beacon);

/* This function computes the displacements of the observed robots from a matrix
 * of DisplacementAccumulator. The matrix must have N_ROBOTS rows, one for each
 * robot in the swarm, and a number of columns equal to the time window used to
 * compute the displacement. The results of the displacement computations will
 * be inserted in an array of length equal to the number of robots in the swarm.
 * The measure of the time must be provided by the caller function.
 *
 * Note about the implementation: the second dimension of the matrix (the number
 * of columns) is not known at compile time because it depends on the caller
 * function. Thus, selection of the row using square brackets is impossible
 * because the function is not able to determine the offset between the address
 * of a row and the address of the subsequent row, that is equal to the size of
 * the struct multiplied by the number of rows. This problem has been resolved
 * substituting the square brackets with the correct arithmetic function applied
 * to the pointer, which uses TIME_WINDOW and can be evaluated at runtime.
 */
void computeDisplacementsMatrix(
    CCI_RangeAndBearingSensor::TReadings& tRABReads,
    const CCI_DifferentialSteeringSensor::SReading& tDifReads,
    float* lastWheelVel, DisplacementAccumulator* matrix,
    CVector2* displacements, unsigned int TIME_WINDOW, unsigned int time,
    std::string id_str);

/* This function computes the transition and the rotation of the robot in the
 * last control step. The variable initialOmega represents the angle between the
 * heading of the robot at the start of the control step and the vertical axis
 * of the reference frame. The velocities are considered to be in cm/s, the
 * distances in cm and the angles in radians.
 */
void computeOwnMovement(float velLeftWheel, float velRightWheel,
                        float lastVelLeftWheel, float lastVelRightWheel,
                        float initialOmega, float& deltaOmega, float& deltaX,
                        float& deltaY);

/* This function takes a string and replaces all the characters equal to
 * characterToFind with the character characterToReplace. It returns the number
 * of substitutions done.
 */
unsigned int stringReplace(std::string& string, char characterToFind,
                           char characterToReplace);

#endif
