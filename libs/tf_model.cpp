#include <Python.h>
#include <string>

const string model_path = "/home/usernamix/Workspace/src/hyperion/hyperion_python/model";

TFmodel::TFmodel() {
	// setup tf model to use for prediction.
	PyObject *pName, *pModule, *pFunc;
    PyObject *pArgs, *pValue;
	Py_Initialize();
    pName = PyString_FromString("module");

	pModule = PyImport_Import(pName);
    Py_DECREF(pName);

    if (pModule != NULL) {
		pFunc = PyObject_GetAttrString(pModule, "function");
	}

}

Eigen::Vector4d TFmodel::predict() {
	// use pretrained tf model for prediction 

}
