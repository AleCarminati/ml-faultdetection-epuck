from os import listdir, remove
from os.path import isfile

import numpy as np
import pandas as pd

from sklearn.preprocessing import LabelEncoder

booleanFeatures = True
foldNumber = 9
behavior= "aggr"

# Find all the files in the directory.
f = []
prefix = "./"+behavior+"_fold_"+str(foldNumber)+"/"
for possibleFile in listdir(prefix):
  if isfile(prefix+possibleFile) :
    f.append(prefix+possibleFile)

# Read the files and put them in different datasets, based on the fault type.
print("Reading files...")
faults = ["pmin", "pmax", "prnd", "rofs", "lact", "ract", "bact"]
separateDatasets = dict(zip(faults, [pd.DataFrame()]*len(faults)))
for fault in faults:
  for file in f:
    if(fault in file):
      separateDatasets[fault] = separateDatasets[fault].append(pd.read_csv(file, delimiter=';').dropna(), ignore_index=True)

# Remove the inconsistent rows.
print("\nRemoving inconsistent rows...")
eliminatedRows = 0
if not(booleanFeatures): 
  for fault in faults:
      for i in range(0, 10):
        # The inconsistency limit of the distance traveled is 52 because it must
        # account for the noise of observation. The nominal limit, without noise,
        # would be 50. 
        inconsistentIndexes = separateDatasets[fault].index[(separateDatasets[fault]['distance_traveled_t-'+str(i)]>52).add(separateDatasets[fault]['min_neighbor_distance_t-'+str(i)]>separateDatasets[fault]['avg_neighbor_distance_t-'+str(i)])]
        eliminatedRows += len(inconsistentIndexes)
        separateDatasets[fault] = separateDatasets[fault].drop(inconsistentIndexes)
print("Number of inconsistent rows: %d"%(eliminatedRows))

# Balance the number of faulty rows for each type of fault.
print("\nBalancing the type of faults...")
minFaultRows = -1
for fault in faults:
  datasetFaultRows = separateDatasets[fault][separateDatasets[fault]["fault_probability"] == 1]
  if(minFaultRows==-1 or minFaultRows>datasetFaultRows.shape[0]):
    minFaultRows = datasetFaultRows.shape[0]
print("Number of faulty rows after balancing between the fault types:")
for fault in faults:
  droppedRows = 0
  datasetFaultRows = separateDatasets[fault][separateDatasets[fault]["fault_probability"] == 1]
  while(datasetFaultRows.shape[0] > minFaultRows):
    faultySituations = datasetFaultRows[["id_experiment", "control_step", "observed_robot"]].drop_duplicates()
    situationToDrop = faultySituations.loc[np.random.choice(faultySituations.index, 1)]
    rowsBeforeDropping = separateDatasets[fault].shape[0]
    separateDatasets[fault] = separateDatasets[fault].drop(
      separateDatasets[fault].loc[np.logical_and(separateDatasets[fault]["observed_robot"] == situationToDrop["observed_robot"].iloc[0],
      np.logical_and(separateDatasets[fault]["id_experiment"] == situationToDrop["id_experiment"].iloc[0], 
      separateDatasets[fault]["control_step"] == situationToDrop["control_step"].iloc[0]))].index)
    droppedRows += rowsBeforeDropping - separateDatasets[fault].shape[0]
    datasetFaultRows = separateDatasets[fault][separateDatasets[fault]["fault_probability"] == 1]
    if datasetFaultRows.shape[0] < 210:
      for file in f:
        remove(file)
      raise Exception(str(datasetFaultRows.shape[0])+" faulty rows.")
  print(fault + ": "+ str(datasetFaultRows.shape[0]) + " "*3 + "=>" + " "*3 + f"{droppedRows:5} rows dropped")

# Balance the percentage of faulty and non-faulty rows.
print("\nBalancing faulty and non-faulty rows...")
totalDataset = pd.DataFrame()
alpha = 1000
t= 1
for fault in faults:
  totalDataset = totalDataset.append(separateDatasets[fault], ignore_index=True)
normalRows = totalDataset[totalDataset["fault_probability"]==0]
droppedRows = 0
while(normalRows.shape[0] > 0.80 * totalDataset.shape[0]):
  normalSituations = normalRows[["id_experiment", "control_step", "observed_robot"]].drop_duplicates()
  situationToDrop = normalSituations.loc[np.random.choice(normalSituations.index, int(np.floor(alpha/np.sqrt(t))))]
  rowsBeforeDropping = totalDataset.shape[0]
  totalDataset = totalDataset.drop(
      totalDataset.loc[np.logical_and(totalDataset["observed_robot"] == situationToDrop["observed_robot"].iloc[0],
      np.logical_and(totalDataset["id_experiment"] == situationToDrop["id_experiment"].iloc[0], 
      totalDataset["control_step"] == situationToDrop["control_step"].iloc[0]))].index)
  droppedRows += rowsBeforeDropping - totalDataset.shape[0]
  normalRows = totalDataset[totalDataset["fault_probability"]==0]
  t+=1
  #print(f"Current percentage of normal rows: {normalRows.shape[0]/totalDataset.shape[0]:2.4%}", end='\r')
print(f"Number of rows after balancing faulty and non-faulty rows: {totalDataset.shape[0]}" + " "*3  + "=>" + " "*3 + f"{droppedRows} rows dropped")

# Use label encoding on the id of the experiment to reduce their value. 
totalDataset['id_experiment'] = LabelEncoder().fit_transform(totalDataset['id_experiment'])

# Write the preprocessed dataset in a new .csv file.
totalDataset.to_csv("../2. preprocessed_folds/"+behavior+"_fold_"+str(foldNumber)+".csv", index=False)