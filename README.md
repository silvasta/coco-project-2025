# COCO_trafficProject_Assignment

## If you want to run your code on a different machine

### 1. Clone this repository

```bash
git clone https://gitlab.ethz.ch/kmoffat/coco_trafficproject_assignment.git
cd coco_trafficproject_assignment
```

### 2. Install SUMO

Follow the instructions on the official SUMO website to install SUMO v1.23.1:
[SUMO Installation Guide](https://sumo.dlr.de/docs/Installing/index.html)

Note the version number is important and would affect reproducibility.

### 2. Set Up a Virtual Environment

Create and activate a virtual environment to manage dependencies.
The JupyterHub will use python 3.13.3 to test your solution.

**macOS/Linux:**

```bash
python3 -m venv venv
source venv/bin/activate
```

**Windows:**

```cmd
python -m venv venv
venv\Scripts\activate
```

### 3. Install Python Dependencies

Once the virtual environment is active, install the required dependencies:

```bash
pip install -r requirements.txt
pip install jupyterlab
```

---

## Running the Project

```bash
jupyter lab
```

Run the notebook `main.ipynb`

---

## License

This project is licensed under the **Creative Commons Attribution-NonCommercial-NoDerivatives (CC BY-NC-ND)** license.
See the [LICENSE-CC-BY-NC-ND](./LICENSE-CC-BY-NC-ND) file for more details.

---

Run the project
