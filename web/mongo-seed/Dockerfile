FROM mongo

COPY hospitals.json /hospitals.json

ENV MONGO_INITDB_ROOT_USERNAME=admin
ENV MONGO_INITDB_ROOT_PASSWORD=password
ENV MONGO_INITDB_DATABASE=HealthData

CMD mongoimport --host mongodb --db HealthData --collection Hospitals --type json --file /hospitals.json --jsonArray --username $MONGO_INITDB_ROOT_USERNAME --password $MONGO_INITDB_ROOT_PASSWORD --authenticationDatabase admin