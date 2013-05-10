# generate python code for the protobuff definition
protoc -I=messages --python_out=orwell/proxy_simulator/ messages/version1.proto
