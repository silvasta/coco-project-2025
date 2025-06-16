def hello():
    print("student MPC loaded")


def check_inputs(inputs: dict = {}):
    for name, input in inputs.items():
        print(f"{name} - {input}")
        print()
    print()
