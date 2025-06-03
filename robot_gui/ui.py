# source ~/Desktop/robot-env/bin/activate
# cd Desktop
import tkinter as tk
import sounddevice as sd
from scipy.io.wavfile import write
import speech_recognition as sr
import numpy as np
from scipy.io.wavfile import read
import spacy

is_recording = False
recording_stream = None
audio_data = []
nlp = spacy.load("en_core_web_sm")

def beep(frequency=440, duration=0.2, sample_rate=44100):
    t = np.linspace(0, duration, int(sample_rate * duration), False)
    tone = np.sin(frequency * 2 * np.pi * t)
    audio = tone.astype(np.float32)
    sd.play(audio, sample_rate)
    sd.wait()


def record_audio():
    global is_recording, recording_stream, audio_data

    if not is_recording:
        print("Beep: start recording")
        beep()
        
        print("Recording started... Press again to stop.")
        is_recording = True
        audio_data = []

        # Start a non-blocking stream
        def callback(indata, frames, time, status):
            if status:
                print("Recording error:", status)
            audio_data.append(indata.copy())

        recording_stream = sd.InputStream(samplerate=44100, channels=1, callback=callback)
        recording_stream.start()

        # Update button label
        record_button.config(text="Stop Recording")

    else:
        print("Beep: end recording")
        beep(frequency=660)
        print("Recording stopped.")
        is_recording = False

        recording_stream.stop()
        recording_stream.close()

        # Concatenate and save
        full_audio = np.concatenate(audio_data, axis=0)
        audio_int16 = (full_audio * 32767).astype('int16')
        write('audio.wav', 44100, audio_int16)

        print("Audio saved as audio.wav")

        # Reset button label
        record_button.config(text="Record Audio Description")


def send_request():
    print("Sending processed request to robot...")

    recognizer = sr.Recognizer()
    audio_file = "audio.wav"

    try:
        with sr.AudioFile(audio_file) as source:
            audio_data = recognizer.record(source)
            text = recognizer.recognize_google(audio_data)
            print("Transcribed Text:", text)
            
            # NLP-based Keyword Extraction
            doc = nlp(text)
            verbs = [token.text for token in doc if token.pos_ == "VERB"]
            nouns = [token.text for token in doc if token.pos_ == "NOUN"]
            print("Extracted Verbs:", verbs)
            print("Extracted Nouns:", nouns)

            with open("parsed_keywords.txt", "w") as f:
                f.write("Verbs: " + ", ".join(verbs) + "\n")
                f.write("Nouns: " + ", ".join(nouns) + "\n")

    except sr.UnknownValueError:
        print("Could not understand the audio.")
    except sr.RequestError as e:
        print(f"Speech recognition request failed: {e}")
    except FileNotFoundError:
        print("Audio file not found. Please record first.")

def play_audio():
    try:
        fs, data = read('audio.wav')
        # Normalize if it's int16 format
        if data.dtype == np.int16:
            data = data.astype(np.float32) / 32767
        sd.play(data, fs)
        sd.wait()
        print("Playback finished.")
    except Exception as e:
        print("Error during playback:", e)

def dummy_action(action_name):
    print(f"{action_name} button pressed")

# Create the main window
root = tk.Tk()
root.title("Assistive Robot UI")
root.geometry("700x400")  # Adjusted height since fewer buttons
root.configure(bg="white")

container = tk.Frame(root, bg="white")
container.pack(expand=True)
container.place(relx=0.5, rely=0.5, anchor='center')

# Title label
title_label = tk.Label(container, text="Assistive Robot Control Panel", font=("Arial", 22, "bold"), bg="white")
title_label.pack(pady=40)

button_font = ("Arial", 15)
button_width = 35
button_height = 2

# Buttons
record_button = tk.Button(container, width=button_width, height=button_height, font=button_font)
record_button.config(text="Record Audio Description", command=record_audio)
record_button.pack(pady=30)

send_button = tk.Button(container, text="Send Request to Robot", command=send_request, width=button_width, height=button_height, font=button_font)
send_button.pack(pady=30)

play_button = tk.Button(container, text="Play Back Recording", command=play_audio,
                        width=button_width, height=button_height, font=button_font)
play_button.pack(pady=30)

# Start the GUI loop
root.mainloop()