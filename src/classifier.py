import pandas as pd
import numpy as np
from sklearn.model_selection import train_test_split
from sklearn.svm import SVC
from sklearn.metrics import classification_report, confusion_matrix
from sklearn.externals import joblib

# Read dataset
data = pd.read_csv("~/catkin_ws/src/room_classification/datasets/last_data.csv")
# Split attributes and labels
x = data.drop('Class', axis=1)
y = data['Class']
# Split training and testing
x_train, x_test, y_train, y_test = train_test_split(x,y, test_size=0.3)
# print(set(y_train))
# Train model
model = SVC(kernel='linear')
model.fit(x_train, y_train)
# # Predict
y_pred = model.predict(x_test)
print(confusion_matrix(y_test, y_pred))

# Save model
filename = 'temp.sav'
joblib.dump(model, filename)
# # Reevaluate saved model
# model2 = joblib.load(filename)
# results = model2.predict(x)
# print(confusion_matrix(y, results))
