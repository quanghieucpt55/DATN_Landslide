import time
import serial
import RPi.GPIO as GPIO
from tb_gateway_mqtt import TBGatewayMqttClient
import logging.handlers
import json
import threading

#Cau hinh TB
ACCESS_TOKEN = "mtvVa4sdzjp0GsipS3m8"
THINGSBOARD_SERVER = 'quanghieutb.shop'

logging.getLogger("tb_connection").setLevel(logging.ERROR)
 
#tao client MQTT de ket noi TB
client = TBGatewayMqttClient(THINGSBOARD_SERVER, username=ACCESS_TOKEN)

# Pin cho LED
LED_RED = 25
LED_YELLOW = 8
LED_GREEN = 7
# Pin cho nut nhan
BUTTON = 12
# Pin cho Buzzer
BUZZER = 1

# Cau hinh GPIO
GPIO.setwarnings(False)
GPIO.setmode(GPIO.BCM)
GPIO.setup(23, GPIO.OUT)  # Chan M0 LoRa
GPIO.setup(24, GPIO.OUT)  # Chan M1 LoRa
GPIO.setup(LED_RED, GPIO.OUT)
GPIO.setup(LED_YELLOW, GPIO.OUT)
GPIO.setup(LED_GREEN, GPIO.OUT)
GPIO.setup(BUZZER, GPIO.OUT)
GPIO.setup(BUTTON, GPIO.IN, pull_up_down=GPIO.PUD_UP)  # Cau hinh nut nhan

# che do truyen nhan lora 
GPIO.output(23, GPIO.LOW)  # M0 = LOW
GPIO.output(24, GPIO.LOW)  # M1 = LOW

# Khoi tao cong serial cho giao tiep 
lora = serial.Serial('/dev/serial0', baudrate=4800, parity=serial.PARITY_NONE, stopbits=serial.STOPBITS_ONE, bytesize=serial.EIGHTBITS)

time.sleep(1)

#Danh sach cac EndNode
node_ids = [0x01, 0x02, 0x03] 

# Bien toan cuc de luu attributes
shared_attributes = {
    "set_time": 3,
    "sm_lower": 0,
    "sm_upper": 0,
    "tilt_lower": 0,
    "tilt_upper": 0,
    "buzzer": False
}

def connect_to_thingsboard():
    while True:
        try:
            client.connect()
            print("Connected to ThingsBoard")

            # Request attributes khi ket noi
            client._client.subscribe("v1/devices/me/attributes/response/1")
            client._client.publish("v1/devices/me/attributes/request/1", json.dumps({"sharedKeys": "sm_lower,sm_upper,tilt_lower,tilt_upper,buzzer,set_time"}), qos=1)

            # Subscribe topic attributes
            client._client.subscribe("v1/devices/me/attributes")
            print("Subscribed to shared attributes topic")
            break
        except Exception as e:
            print(f"Failed to connect to ThingsBoard: {e}")
            time.sleep(5)  # Retry after a short delay


# Ham xu ly khi co attributes cap nhat
def handle_attributes_update(attributes):
    global shared_attributes
    for key, value in attributes.items():
        if key in ["sm_lower", "sm_upper", "tilt_lower", "tilt_upper","set_time"]:
            try:
                shared_attributes[key] = float(value)
            except ValueError:
                print(f"Failed to convert {key} with value {value} to float. Keeping as string.")
        elif key == "buzzer":
            shared_attributes[key] = value
    print("Updated shared attributes:", shared_attributes)

# Ham xu ly khi co tin nhan MQTT
def on_message(client,userdata, message):
    try:
        payload = message.payload.decode("utf-8")
        data = json.loads(payload)

        if "shared" in data:
            print("Receibved shared attributes 1:", data)
            handle_attributes_update(data["shared"])
        elif any(key in data for key in ["maxPayloadSize", "rateLimits", "gatewayRateLimits"]):
            return  
        else:
            print("Receibved shared attributes 2:", data)
            handle_attributes_update(data)
            # Dieu khien buzzer
        GPIO.output(BUZZER, GPIO.HIGH if shared_attributes["buzzer"] else GPIO.LOW)
    except Exception as e:
        print(f"Error processing MQTT message: {e}")


def control_leds(sm, tilt):
    # Dieu khien LED 
    if sm > shared_attributes["sm_upper"] or tilt > shared_attributes["tilt_upper"]:
        GPIO.output(LED_RED, GPIO.HIGH)
        GPIO.output(LED_YELLOW, GPIO.LOW)
        GPIO.output(LED_GREEN, GPIO.LOW)

    elif shared_attributes["sm_lower"] < sm < shared_attributes["sm_upper"] or shared_attributes["tilt_lower"] < tilt < shared_attributes["tilt_upper"]:
        GPIO.output(LED_RED, GPIO.LOW)
        GPIO.output(LED_YELLOW, GPIO.HIGH)
        GPIO.output(LED_GREEN, GPIO.LOW)
    else:
        GPIO.output(LED_RED, GPIO.LOW)
        GPIO.output(LED_YELLOW, GPIO.LOW)
        GPIO.output(LED_GREEN, GPIO.HIGH)
    if tilt > shared_attributes["tilt_upper"]:
        GPIO.output(BUZZER, GPIO.HIGH)
def button_control_buzzer(chanel):
    global shared_attributes
    print("Button pressed!")
    shared_attributes["buzzer"] = not shared_attributes["buzzer"]  # Dao trang thai buzzer
    GPIO.output(BUZZER, GPIO.HIGH if shared_attributes["buzzer"] else GPIO.LOW)

    # Cap nhat attributes len ThingsBoard
    payload = json.dumps({"buzzer": shared_attributes["buzzer"]})
    print("payload: ", payload)
    client._client.publish("v1/devices/me/attributes", payload, qos=1)

# Dang ky callback cho nut nhan
GPIO.add_event_detect(BUTTON, GPIO.FALLING, callback=button_control_buzzer, bouncetime=500)  # Bouncetime de tranh double trigger
    
def send_lora_data_to_tb(node_id, data1, data2, data3):
    client.gw_connect_device(node_id)

    #tao payload
    payload =   { 
                "sm": data1,
                "tilt": data2,
                "temp": data3
                }
    
    # Gui du lieu len tb
    result = client.gw_send_telemetry(node_id, payload)
    print(f"Data from {node_id} sent to TB successfully")

def calculate_crc(data):
    # Tinh CRC
    crc = 0
    for byte in data:
        crc ^= byte
    return crc

def decode_and_validate(data):
    try:
        # Kiem tra do dai goi tin
        if len(data) != 8:
            print("Invalid packet length!")
            return None

        # Tach cac truong du lieu tu goi tin
        node_id = data[0]
        soil_moisture = (data[1] << 8 | data[2]) / 10.0  # Soil moisture (2 bytes)
        tilt = (data[3] << 8 | data[4]) / 10.0                            # Tilt angle (2 byte)
        temp = (data[5] << 8 | data[6]) / 10.0          # Temperature (2 bytes)
        crc_received = data[7]                          # CRC tu goi tin

        # Tinh CRC
        crc_calculated = calculate_crc(data[:7])
        if crc_calculated != crc_received:
            print("CRC mismatch! Data corrupted.")
            return None

        # Tra ve du lieu giai ma
        return {
            "id": f"Node_{node_id}",
            "sm": soil_moisture,
            "tilt": tilt,
            "temp": temp,
        }
    except Exception as e:
        print(f"Error decoding data: {e}")
        return None

lora_lock = threading.Lock()

def receive_and_process_lora():
    with lora_lock:
        try:
            # Kiem tra du lieu tu LoRa
            if lora.inWaiting() >= 8:
                # Doc du lieu tu LoRa
                raw_data = lora.read(lora.inWaiting())

                # Giai ma va kiem tra CRC
                decoded_data = decode_and_validate(raw_data)
                if decoded_data:
                    node_id = decoded_data["id"]
                    sm = decoded_data["sm"]
                    tilt = decoded_data["tilt"]
                    temp = decoded_data["temp"]

                    # In du lieu da giai ma
                    print(f"node_id = {node_id}, sm = {sm}, tilt = {tilt}, temp = {temp}")

                    # Dieu khien LED va Buzzer
                    control_leds(sm, tilt)

                    # Gui du lieu len ThingsBoard
                    send_lora_data_to_tb(node_id, sm, tilt, temp)
                else:
                    print("No data received from LoRa")
        except Exception as e:
            print(f"Failed to read data")

def send_command_to_node(node_id):
    with lora_lock:
        try:
            lora.write(bytes([node_id]))
            print(f"Command sent to Node {node_id:02X}")
        except Exception as e:
            print(f"Failed to send command to Node {node_id:02X}: {e}")

stop_threads = False

def receive_thread():
    while not stop_threads:
        receive_and_process_lora()

def command_thread():
    while not stop_threads:
        for node_id in node_ids:
            send_command_to_node(node_id)
            time.sleep(shared_attributes["set_time"])

def main():
    global stop_threads
    try:
        connect_to_thingsboard()
        client._client.on_message = on_message

        t1 = threading.Thread(target=receive_thread, daemon=True)
        t2 = threading.Thread(target=command_thread, daemon=True)

        t1.start()
        t2.start()

        t1.join()
        t2.join()
    except KeyboardInterrupt:
        print("Exiting Program...")
        stop_threads = True  
        t1.join()  
        t2.join()
    except Exception as e:
        print(f"Error: {e}, Exiting Program")
    finally:
        if lora.is_open:
            lora.close()
        GPIO.cleanup()
        client.disconnect()

if __name__ == "__main__":
    main()