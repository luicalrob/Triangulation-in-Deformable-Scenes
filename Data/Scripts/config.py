#!/usr/bin/env python3

      
syncolon_values = {
    "Pair": [
        "150-2-160", 
        "270-2-300",
        "70-3-90", 
        "105-2-115"
    ],
    "Triangulation": ["FarPoints"],
    "Level": ["seq0", "seq1", "seq2", "seq3", "seq4", "seq5"],
    "Checks": ["checks", "no_checks"],
    "Experiment": range(66, 71),
}

syncolon_level_types = {
    "seq0": ["checks", "no_checks"],
    "seq1": ["no_checks"],
    "seq2": ["no_checks"],
    "seq3": ["no_checks"],
    "seq4": ["no_checks"],
    "seq5": ["no_checks"]
}

def setSyncolonParameters(pair, level):
    """
    Args:
        pair (string)
        level (string)
    
    Returns:
        dict: parameters for .yalm
    """
    # Diccionario de configuración
    experiment_config = {
        "70-3-90_seq0": {"Hdist": 30, "window": 30, "maxDepth": 0.1, "minParallax": 1.8},
        "70-3-90_seq1": {"Hdist": 30, "window": 30, "maxDepth": 0.1, "minParallax": 1.8},
        "70-3-90_seq2": {"Hdist": 35, "window": 15, "maxDepth": 0.1, "minParallax": 1.8},
        "70-3-90_seq3": {"Hdist": 35, "window": 15, "maxDepth": 0.1, "minParallax": 1.8},
        "70-3-90_seq4": {"Hdist": 35, "window": 15, "maxDepth": 0.1, "minParallax": 1.8},
        "70-3-90_seq5": {"Hdist": 35, "window": 20, "maxDepth": 0.1, "minParallax": 1.8},

        "105-2-115_seq0": {"Hdist": 30, "window": 25, "maxDepth": 0.1, "minParallax": 1.8},
        "105-2-115_seq1": {"Hdist": 30, "window": 25, "maxDepth": 0.1, "minParallax": 1.8},
        "105-2-115_seq2": {"Hdist": 30, "window": 30, "maxDepth": 0.1, "minParallax": 1.8},
        "105-2-115_seq3": {"Hdist": 35, "window": 30, "maxDepth": 0.1, "minParallax": 1.8},
        "105-2-115_seq4": {"Hdist": 35, "window": 30, "maxDepth": 0.1, "minParallax": 1.8},
        "105-2-115_seq5": {"Hdist": 35, "window": 30, "maxDepth": 0.1, "minParallax": 1.8},

        "150-2-160_seq0": {"Hdist": 30, "window": 30, "maxDepth": 0.1, "minParallax": 2.5},
        "150-2-160_seq1": {"Hdist": 30, "window": 30, "maxDepth": 0.1, "minParallax": 2.5},
        "150-2-160_seq2": {"Hdist": 30, "window": 30, "maxDepth": 0.1, "minParallax": 2.5},
        "150-2-160_seq3": {"Hdist": 30, "window": 30, "maxDepth": 2.0, "minParallax": 2.5},
        "150-2-160_seq4": {"Hdist": 30, "window": 35, "maxDepth": 0.1, "minParallax": 2.5},
        "150-2-160_seq5": {"Hdist": 30, "window": 35, "maxDepth": 0.1, "minParallax": 2.5},

        "270-4-300_seq0": {"Hdist": 25, "window": 25, "maxDepth": 0.1, "minParallax": 2.5},
        "270-4-300_seq1": {"Hdist": 25, "window": 25, "maxDepth": 0.1, "minParallax": 2.5},
        "270-4-300_seq2": {"Hdist": 35, "window": 25, "maxDepth": 0.1, "minParallax": 2.5},
        "270-4-300_seq3": {"Hdist": 35, "window": 25, "maxDepth": 0.1, "minParallax": 2.5},
        "270-4-300_seq4": {"Hdist": 35, "window": 25, "maxDepth": 0.1, "minParallax": 2.5},
        "270-4-300_seq5": {"Hdist": 35, "window": 55, "maxDepth": 0.1, "minParallax": 2.5},
    }
    
    try:
        return experiment_config[pair+"_"+level]
    except KeyError:
        raise ValueError("The pair must be one the determinated previously.")

drunkard_values = {
    "Pair": [
        "320_00000_1975-1983", 
        "320_00000_2500-2513",
        "1024_00000_1229-1236",
        "1024_00001_110-120"
    ],
    "Triangulation": ["InRays", "TwoPoints", "FarPoints"],
    "Level": ["level0", "level1", "level2", "level3"],
    "Checks": ["checks", "no_checks"],
    "Experiment": range(26, 31),
}

drunkard_level_types = {
    "level0": ["checks", "no_checks"],
    "level1": ["no_checks"],
    "level2": ["no_checks"],
    "level3": ["no_checks"]
}

def setDrunkardParameters(pair):
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
