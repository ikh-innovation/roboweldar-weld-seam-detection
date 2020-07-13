# Dockerfile - this is a comment. Delete me if you want.
FROM python:3.6
COPY requirements.txt /tmp/
RUN apt-get update
RUN apt-get --yes --force-yes install libgl1-mesa-glx
RUN pip install -r /tmp/requirements.txt
WORKDIR /app
COPY seam-detection/prediction.py .
COPY flask-server.py .
ENTRYPOINT ["python"]
CMD ["./flask-server.py"]
