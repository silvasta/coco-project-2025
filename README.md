# Computational Control (CoCo) Traffic Project

## If you want to run your this code

### 1. Clone this repository

```bash
git clone https://github.com/silvasta/coco-project-2025.git
```

### 2. Install SUMO

Follow the instructions on the official SUMO website to install SUMO v1.23.1:
[SUMO Installation Guide](https://sumo.dlr.de/docs/Installing/index.html)

Note the version number is important and would affect reproducibility.

### 2. Set Up a Virtual Environment Management

#### Install uv

```bash
# best and must have package manager for python
curl -LsSf <https://astral.sh/uv/install.sh> | sh
# (link valid at 2025.07.22, check if still proper)
```

### 3. Install Python Dependencies

```bash
cd coco-project-2025
# create environment, install python version and dependencies
uv sync
```

reads `pyproject.toml` and `uv.lock` $\rightarrow$ creates environment and resolves all dependencies

#### General uv info

```bash
# if you actually need stuff from requirements.txt
uv pip install -r requirements.txt
# to do it proper, add important libraries with
uv add {important_library}
# or add them manually in pyproject.toml
```

## Running the Project

```bash
# run student code
uv run silvasta.py
# run original code
uv run main.py
```

**Run the notebook** Use JupyterLab (f.e. available in snap app store)

#### Jupyter with uv

##### Assuming no uv environment exists

```bash
# create environment
uv init
```

##### Otherwise or after creation of new uv project

```bash
# connect to Jupyter
uv add --dev ipykernel
uv run ipyhon kernel install --user --env .venv --name sysco # or any other name
```

---

## License

This project is licensed under the **Creative Commons Attribution-NonCommercial-NoDerivatives (CC BY-NC-ND)** license.
See the [LICENSE-CC-BY-NC-ND](./LICENSE-CC-BY-NC-ND) file for more details.
