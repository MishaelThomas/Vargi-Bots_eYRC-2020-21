from multiprocessing.dummy import Pool
import requests
import paho.mqtt.client as mqtt #import the client1
import time


# -----------------  MQTT SUB -------------------
def mqtt_subscribe_thread_start(arg_callback_func, arg_broker_url, arg_broker_port, arg_mqtt_topic, arg_mqtt_qos):
    try:
        mqtt_client = mqtt.Client()
        mqtt_client.on_message = arg_callback_func
        mqtt_client.connect(arg_broker_url, arg_broker_port)
        mqtt_client.subscribe(arg_mqtt_topic, arg_mqtt_qos)
        time.sleep(1) # wait
        # mqtt_client.loop_forever() # starts a blocking infinite loop
        mqtt_client.loop_start()    # starts a new thread
        return 0
    except:
        return -1

#----------------------------------http reuest------------------------------------------
# defining our sheet name in the 'id' variable and the the column where we want to update the value
def spreadsheet_write(URl,**kwargs):
    URL = URl
    response = requests.get(URL, params=kwargs)
    return response.content
