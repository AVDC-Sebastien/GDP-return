import Remote_Connection.Server as Server


# Start a local Server on port 65000
HOST, PORT = '0.0.0.0', 65000
print(f"Opening the local server in the port: {PORT} ")
server = Server(HOST,PORT)

