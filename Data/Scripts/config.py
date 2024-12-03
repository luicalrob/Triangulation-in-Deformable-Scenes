#!/usr/bin/env python3

default_values = {
    "Model": [
        "ARAP", 
        "ARAP_not_scaled_depth",
        "ARAP_depth",
        "ARAP_depth_3mm",
        "ARAP_depth_8mm",
        "ARAP_depth_onlyTriang", 
        "Elastic", 
        "HyperElasticOdgen", 
        "ARAP_NoGlobal", 
        "ARAP_OneSet"
    ],
    "Triangulation": ["InRays", "TwoPoints", "FarPoints"],
    "Depth": [20, 80, 150],
    "Shape": ["Planar", "Gradual"],
    "ExperimentType": range(1, 7),
    "Experiment": range(1, 6),
}

shape_experiment_types = {
    "Planar": [1, 2, 3, 4, 5, 6],
    "Gradual": [2, 3, 5, 6],
}

def setExperiment(experiment_type):
    """
    Args:
        experiment_type (int): The experiment type (1 a 6).
    
    Returns:
        dict: gaussianMov and rigidMov values
    """
    # Diccionario de configuración
    experiment_config = {
        1: {"gaussian": 2.5, "rigid": 0},
        2: {"gaussian": 0, "rigid": 2.5},
        3: {"gaussian": 2.5, "rigid": 2.5},
        4: {"gaussian": 10, "rigid": 0},
        5: {"gaussian": 0, "rigid": 10},
        6: {"gaussian": 10, "rigid": 10},
    }
    
    try:
        return experiment_config[experiment_type]
    except KeyError:
        raise ValueError("The type of experiment must be between 1 and 6.")
