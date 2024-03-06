# import json
# import os

# # Function to check if a JSON file exists, open and read it, or create a new one
# def handle_json_file(file_path, default_data=None):
#     if os.path.exists(file_path):
#         # If the file exists, open and read it
#         with open(file_path, 'r') as file:
#             data = json.load(file)
#         print(f"Existing JSON file found and loaded: {file_path}")
#     else:
#         # If the file does not exist, create a new one with default data
#         data = default_data or {}
#         with open(file_path, 'w') as file:
#             json.dump(data, file, indent=2)
#         print(f"New JSON file created: {file_path}")

#     return data
import logging
test= "zegg"



logging.basicConfig(filename="GDP_retrun_server.log",level=logging.DEBUG,format='%(asctime)s - %(levelname)s - %(message)s', datefmt='%d-%b-%y %H:%M:%S')
logging.debug(f"This is just a harmless debug message {test}") 
logging.info("This is just an information for you") 
logging.warning("OOPS!!!Its a Warning") 
logging.error("Have you try to divide a number by zero") 
logging.critical("The Internet is not working....") 
logging.info("test")
print("\033[93maefeg\033[00m".split("==")[-1])