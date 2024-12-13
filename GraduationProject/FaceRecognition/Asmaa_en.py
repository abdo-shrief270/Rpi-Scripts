import face_recognition
import pickle
import os

# Directory with images of known faces
known_faces_dir = "faces"
known_face_encodings = []
known_face_names = []

for image_name in os.listdir(known_faces_dir):
    image_path = os.path.join(known_faces_dir, image_name)
    image = face_recognition.load_image_file(image_path)
    encoding = face_recognition.face_encodings(image)[0]
    
    known_face_encodings.append(encoding)
    known_face_names.append(os.path.splitext(image_name)[0])  # Use the filename as the name

# Save the encodings to a file
data = {"encodings": known_face_encodings, "names": known_face_names}
with open("encodings.pickle", "wb") as f:
    f.write(pickle.dumps(data))
