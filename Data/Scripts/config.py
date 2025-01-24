#!/usr/bin/env python3

real_values = {
    "Pair": [
        "320_00000_1975-1983", 
        "320_00000_2500-2513",
        "1024_00000_1229-1236",
        "1024_00001_110-120"
    ],
    "Triangulation": ["InRays", "TwoPoints", "FarPoints"],
    "Level": ["level0", "level1", "level2", "level3"],
    "Checks": ["checks", "no_checks"],
    "Experiment": range(11, 16),
}

level_types = {
    "level0": ["checks", "no_checks"],
    "level1": ["no_checks"],
    "level2": ["no_checks"],
    "level3": ["no_checks"]
}

def setParameters(pair):
    """
    Args:
        pair (string)
    
    Returns:
        dict: parameters for .yalm
    """
    # Diccionario de configuración
    experiment_config = {
        "1024_00000_1229-1236": {"Hdist": 30, "window": 75, "maxDepth": 2.5, "minParallax": 1.9},
        "1024_00001_110-120": {"Hdist": 30, "window": 120, "maxDepth": 6.0, "minParallax": 0.6},
        "320_00000_2500-2513": {"Hdist": 35, "window": 40, "maxDepth": 3.5, "minParallax": 1.6},
        "320_00000_1975-1983": {"Hdist": 35, "window": 40, "maxDepth": 3.5, "minParallax": 0.6},
    }
    
    try:
        return experiment_config[pair]
    except KeyError:
        raise ValueError("The pair must be one the determinated previously.")

synthetic_values = {
    "Model": [
        "ARAP", 
        "ARAP_not_scaled_depth",
        "ARAP_depth_1mm",
        "ARAP_depth_3mm",
        "ARAP_depth_8mm",
        "ARAP_depth_onlyTriang", 
        "Elastic", 
        "HyperElasticOdgen", 
        "ARAP_NoGlobal", 
        "ARAP_OneSet"
    ],
    "Triangulation": ["InRays", "TwoPoints", "FarPoints"],
    "Depth": [20],
    "Shape": ["Planar", "Gradual"],
    "ExperimentType": range(1, 7),
    "Experiment": [2, 3],
}

shape_syn_experiment_types = {
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
