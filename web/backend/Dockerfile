FROM python:3.9

WORKDIR /app

COPY . .

RUN pip install --upgrade pip
RUN pip3 install wheel setuptools pip --upgrade
RUN pip install Cmake
RUN pip install -r requirements.txt

EXPOSE 8000

CMD ["uvicorn", "main:app", "--host", "0.0.0.0", "--port", "8000", "--reload"]
