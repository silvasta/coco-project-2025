

class NetworkNameError(ValueError):
    def __init__(self,name):
        self.message = f"The network {name} does not exist, please create it in the /dep/sumo_files folder."
        super().__init__(self.message)
        
class NetworkCreationError(ValueError):
    def __init__(self,name):
        self.message = f"The network could not be created, please check the folder /dep/sumo_files/{name}"
        super().__init__(self.message)
        
class ModNameError(ValueError):
    def __init__(self,name):
        self.message = f"The modality {name} does not exist, please select one of the available modalities."
        super().__init__(self.message)

class RegionLabelsError(ValueError):
    def __init__(self,name):
        self.message = f"Could not find the labels.npy file, please create it in /dep/sumo_files/{name}/regions/ "
        super().__init__(self.message)

class TLSFileError(ValueError):
    def __init__(self,name):
        self.message = f"Could not find the TLS.json file, please create it in /dep/sumo_files/{name}/control/ "
        super().__init__(self.message)
        
class MFDFileError(ValueError):
    def __init__(self,name):
        self.message = f"Could not find the MFD .npy files, please create them in /dep/sumo_files/{name}/regions/ "
        super.__init__(self.message)

class HankelFileError(ValueError):
    def __init__(self,name,modname):
        self.message = f"Could not find the Hankel .pkl file, please create it in /dep/sumo_files/{name}/control/{modname}/ "
        super().__init__(self.message)

class ParamsFileError(ValueError):
    def __init__(self,name,namecon,modname):
        self.message = f"Could not find the params_{namecon}.json file, please create it in /dep/sumo_files/{name}/control/{modname}/ "
        super().__init__(self.message)

class ControllerNameError(ValueError):
    def __init__(self,name):
        self.message = f"The controller {name} does not exist, please select one of the available controllers."
        super().__init__(self.message)

class ControllerCreationError(ValueError):
    def __init__(self,name):
        self.message = f"The controller {name} could not be created"
        super().__init__(self.message)

class SimCreationError(ValueError):
    def __init__(self,name):
        self.message = f"The simulation {name} could not be created, please check the folder /dep/sumo_files/{name}"
        super().__init__(self.message)

class SimRuntimeError(ValueError):
    def __init__(self,name):
        self.message = f"The simulation {name} incurred in a runtime error"
        super().__init__(self.message)