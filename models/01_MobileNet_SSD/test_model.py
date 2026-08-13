import tensorflow as tf

interpreter = tf.lite.Interpreter(model_path="model/detect.tflite")
interpreter.allocate_tensors()

print("MODEL INPUT:")
print(interpreter.get_input_details())

print("\nMODEL OUTPUT:")
for i, output in enumerate(interpreter.get_output_details()):
    print(f"\nOutput {i}:")
    print(output)