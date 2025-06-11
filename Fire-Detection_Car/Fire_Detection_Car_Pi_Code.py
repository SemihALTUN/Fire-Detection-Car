import time
import spidev
import RPi.GPIO as GPIO
from RPLCD.i2c import CharLCD
import sounddevice as sd
from scipy.io.wavfile import write
import numpy as np
import speech_recognition as sr
from time import sleep
from threading import Thread
from gpiozero import Buzzer
from twilio.rest import Client  # Twilio SMS kütüphanesi

# --------------------- TWILIO AYARLARI ---------------------
TWILIO_ACCOUNT_SID = 'YOUR_ACCOUNT_SID'
TWILIO_AUTH_TOKEN = 'YOUR_AUTH_TOKEN'
TWILIO_FROM_NUMBER = '+1XXXXXXXXXX'        # Twilio'dan aldığın numara
TARGET_PHONE_NUMBER = '+90XXXXXXXXXX'      # SMS alacak telefon numarası

def sms_gonder():
    try:
        client = Client(TWILIO_ACCOUNT_SID, TWILIO_AUTH_TOKEN)
        message = client.messages.create(
            body="Alev algılandı! Lütfen müdahale edin.",
            from_=TWILIO_FROM_NUMBER,
            to=TARGET_PHONE_NUMBER
        )
        print("SMS gönderildi:", message.sid)
    except Exception as e:
        print("SMS gönderilemedi:", e)

# --------------------- GPIO VE DİĞER AYARLAR ---------------------
GPIO.setwarnings(False)
GPIO.setmode(GPIO.BCM)

ENA, ENB = 12, 13
IN1, IN2 = 17, 18
IN3, IN4 = 22, 23
motor_pins = [ENA, ENB, IN1, IN2, IN3, IN4]
for pin in motor_pins:
    GPIO.setup(pin, GPIO.OUT)

pwm_ena = GPIO.PWM(ENA, 100)
pwm_enb = GPIO.PWM(ENB, 100)
pwm_ena.start(70)
pwm_enb.start(70)

buzzer = Buzzer(16)

FLAME_PIN = 21
GPIO.setup(FLAME_PIN, GPIO.IN, pull_up_down=GPIO.PUD_UP)

lcd = CharLCD(i2c_expander='PCF8574', address=0x27, cols=16, rows=2)

spi = spidev.SpiDev()
spi.open(0, 0)
spi.max_speed_hz = 1_000_000

def read_adc(ch):
    if not 0 <= ch <= 7:
        raise ValueError("Kanal 0-7 arası olmalı")
    cmd = [1, (8 + ch) << 4, 0]
    resp = spi.xfer2(cmd)
    return ((resp[1] & 3) << 8) | resp[2]

CHANNELS = {
    "MQ2": 2,
    "MQ9": 0,
    "MQ135": 1,
}

THRESHOLD = 250
SAMPLE_TIME = 1.0

command_keywords = {
    'dur': ['dur'],
    'geri_git': ['geri git', 'geri'],
    'ileri_git': ['ileri git', 'ileri'],
    'saga_git': ['sağa git', 'sağa dön', 'sağ'],
    'sola_git': ['sola git', 'sola dön', 'sol'],
    'geri_don': ['geri dön', 'dön']
}

def record_audio(duration=4, fs=22050, filename="mic_input.wav"):
    print(f"🎤 {duration} saniye konuşun...")
    audio = sd.rec(int(duration * fs), samplerate=fs, channels=1)
    sd.wait()
    audio = np.squeeze(audio)
    write(filename, fs, (audio * 32767).astype(np.int16))
    print("✅ Ses kaydedildi.")
    return filename

def speech_to_text_command(audio_path):
    recognizer = sr.Recognizer()
    with sr.AudioFile(audio_path) as source:
        audio = recognizer.record(source)
    try:
        return recognizer.recognize_google(audio, language="tr-TR").lower()
    except sr.UnknownValueError:
        print("❌ Ses anlaşılamadı.")
        return ""
    except sr.RequestError as e:
        print(f"API hatası: {e}")
        return ""

def geri_git(sure=2):
    GPIO.output(IN1, GPIO.LOW)
    GPIO.output(IN2, GPIO.HIGH)
    GPIO.output(IN3, GPIO.LOW)
    GPIO.output(IN4, GPIO.HIGH)
    sleep(sure)

def ileri_git(sure=2):
    GPIO.output(IN1, GPIO.HIGH)
    GPIO.output(IN2, GPIO.LOW)
    GPIO.output(IN3, GPIO.HIGH)
    GPIO.output(IN4, GPIO.LOW)
    sleep(sure)

def saga_don(sure=0.57):
    GPIO.output(IN1, GPIO.LOW)
    GPIO.output(IN2, GPIO.HIGH)
    GPIO.output(IN3, GPIO.HIGH)
    GPIO.output(IN4, GPIO.LOW)
    sleep(sure)

def sola_don(sure=0.63):
    GPIO.output(IN1, GPIO.HIGH)
    GPIO.output(IN2, GPIO.LOW)
    GPIO.output(IN3, GPIO.LOW)
    GPIO.output(IN4, GPIO.HIGH)
    sleep(sure)

def geri_don(sure=1.09):
    print("↩️ Sola dönüyor")
    sola_don(sure)
    dur()

def dur():
    GPIO.output(IN1, GPIO.LOW)
    GPIO.output(IN2, GPIO.LOW)
    GPIO.output(IN3, GPIO.LOW)
    GPIO.output(IN4, GPIO.LOW)

def sensor_loop():
    current_display = ""
    sms_sent = False

    while True:
        readings = {name: read_adc(ch) for name, ch in CHANNELS.items()}
        gas_detected = any(val > THRESHOLD for val in readings.values())
        flame_detected = GPIO.input(FLAME_PIN) == GPIO.LOW

        message_top = "Gaz: VAR" if gas_detected else "Gaz: YOK"
        message_bottom = "Alev: VAR" if flame_detected else "Alev: YOK"

        if flame_detected:
            buzzer.on()
            sleep(2)
            buzzer.off()

            if not sms_sent:
                sms_gonder()
                sms_sent = True
        else:
            sms_sent = False

        full_message = message_top + "|" + message_bottom

        if full_message != current_display:
            lcd.clear()
            lcd.cursor_pos = (0, 0)
            lcd.write_string(message_top.ljust(16))
            lcd.cursor_pos = (1, 0)
            lcd.write_string(message_bottom.ljust(16))
            current_display = full_message
            time.sleep(0.1)

        print(f"{time.strftime('%H:%M:%S')} - {readings} - Gaz:{gas_detected} Alev:{flame_detected}")
        time.sleep(SAMPLE_TIME)

def voice_loop():
    while True:
        audio_file = record_audio()
        komut = speech_to_text_command(audio_file)

        if any(k in komut for k in command_keywords['ileri_git']):
            ileri_git()
            dur()
        elif any(k in komut for k in command_keywords['geri_git']):
            geri_git()
            dur()
        elif any(k in komut for k in command_keywords['saga_git']):
            saga_don()
            dur()
        elif any(k in komut for k in command_keywords['sola_git']):
            sola_don()
            dur()
        elif any(k in komut for k in command_keywords['geri_don']):
            geri_don()
            dur()
        elif any(k in komut for k in command_keywords['dur']):
            dur()
        else:
            print("⚠️ Komut tanınmadı.")
        time.sleep(1)

def main():
    try:
        t1 = Thread(target=sensor_loop)
        t2 = Thread(target=voice_loop)
        t1.start()
        t2.start()
        t1.join()
        t2.join()
    except KeyboardInterrupt:
        print("\n🚨 Program sonlandırıldı.")
    finally:
        pwm_ena.stop()
        pwm_enb.stop()
        GPIO.cleanup()
        spi.close()
        lcd.clear()

if __name__ == "__main__":
    main()