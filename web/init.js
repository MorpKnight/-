db = db.getSiblingDB('HealthData');

db.createCollection("Users");
db.createCollection("Doctors");
db.createCollection("Hospitals");

print("Database and initial collections created successfully.");