# 🚀 FleetGlue - ROS2 + FastAPI + MQTT Deployment Guide

## **📌 Overview**
This is a **Fleet Management System** using:
- 🚀 **FastAPI** for mission management (REST API)
- 🤖 **ROS2 Nodes (RN1 & RN2)** for executing missions with **Action Clients & Servers**
- 💼 **MQTT Broker (Mosquitto)** for real-time mission updates

---

## **⚙️ 1. Prerequisites**
Before running the project, ensure you have:
- **Docker & Docker Compose** installed
- **ROS2 Humble** (if running without Docker)

### **📦 Install Docker & Docker Compose**
For **Ubuntu**:
```sh
sudo apt update
sudo apt install docker.io docker-compose -y
```
For **macOS/Windows**: [Install Docker](https://docs.docker.com/get-docker/)

---

## **🚀 2. Running the Project**
### **💡 Option 1: Using Docker (Recommended)**
Run all services in **Docker**:
```sh
docker-compose up --build
```
💪 **What This Does**
- 🚀 Starts **FastAPI (REST API)**
- 🤖 Starts **RN1 & RN2 ROS2 Nodes**
- 👌 Starts **MQTT Broker (Mosquitto)**

---

### **💡 Option 2: Running Services Manually (Without Docker)**
1️⃣ **Start MQTT Broker (Mosquitto)**
```sh
mosquitto -c mosquitto.conf
```

2️⃣ **Start FastAPI Server**
```sh
python3 app.py
```

3️⃣ **Start ROS2 Nodes (RN1 & RN2)**
```sh
source /opt/ros/humble/setup.bash
source ros2_ws/install/setup.bash

# Run RN1 (Choose MQTT or API version)
ros2 run rn1_action_client rn1_mqtt_node
# OR
ros2 run rn1_action_client rn1_api_node

# Run RN2 (Action Server)
ros2 run rn2_action_server rn2_node
```

---

## **📱 3. Testing the API**
After running the project, use `curl` or Postman to test the API.

### ✅ **POST a New Mission**
```sh
curl -X POST "http://localhost:8000/mission/" \
     -H "Content-Type: application/json" \
     -d '{"id": "1", "command": "3"}'
```

### ✅ **Retrieve a Mission**
```sh
curl -X GET "http://localhost:8000/mission/1"
```

---

## **📃 4. Debugging & Logs**
### 🛠 **Check Running Containers**
```sh
docker ps
```

### 📄 **View Logs**
```sh
docker-compose logs -f rn1
docker-compose logs -f rn2
```

### 🔄 **Restart Everything**
```sh
docker-compose down && docker-compose up --build
```

---

## **👌 5. Deployment to Cloud**
To deploy on **AWS, GCP, or Kubernetes**, you can:
1. **Use AWS ECS** with `docker-compose.yml`
2. **Deploy to Kubernetes** using:
```sh
kubectl apply -f k8s_deployment.yaml
```
3. **Run on AWS EC2** manually with Docker.

🚀 **Coming Soon: Kubernetes & AWS Instructions**

---


## **👨‍💻 Contributors**
Maintained by **Chang-Hyun Mungai**  
📧 Contact: changhyunmungai@gmail.com

---

## **📜 License**
🔓 Open-source under **Apache 2.0 License**

