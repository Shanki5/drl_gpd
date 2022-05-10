from drl_grasping.utils.model_collection_randomizer import ModelCollectionRandomizer
from gym_ignition.scenario import model_wrapper
from gym_ignition.utils.scenario import get_unique_model_name
from scenario import core as scenario
from typing import List


class RandomObject(model_wrapper.ModelWrapper):

    def __init__(self,
                 world: scenario.World,
                 name: str = 'object',
                 position: List[float] = (0, 0, 0),
                 orientation: List[float] = (1, 0, 0, 0),
                 model_paths: str = None,
                 owner: str = 'GoogleResearch',
                 collection: str = 'Google Scanned Objects',
                 server: str = 'https://fuel.ignitionrobotics.org',
                 server_version: str = '1.0',
                 unique_cache: bool = False,
                 reset_collection: bool = False,
                 np_random=None):

        """
        Get a unique model name and the Initial pose of the model
        """
        model_name = get_unique_model_name(world, name)
        initial_pose = scenario.Pose(position, orientation)

        """
        Creating the model collection randomizer using default parameters that include
        1. model_paths - Path containing object model configurations
        2. owner - Copyrights of the different models
        3. collection - Collection of different scanned objects
        4. server - Address of the server to download the object files
        5. server_version - Version of the server
        6. unique_cache - Clearing cache everytime the Randomizer is used (Boolean value)
        7. reset_collection - Reset the object collection everytime the Randomizer is used (Boolean value)
        8. np_random
        """
        model_collection_randomizer = ModelCollectionRandomizer(model_paths=model_paths,
                                                                owner=owner,
                                                                collection=collection,
                                                                server=server,
                                                                server_version=server_version,
                                                                unique_cache=unique_cache,
                                                                reset_collection=reset_collection,
                                                                np_random=np_random)

        """
        Samples a random model from the collection of models using the random_model() function
        Note: using default arguments here
        Inserting the sampled object model into the world
        """
        modified_sdf_file = model_collection_randomizer.random_model()
        ok_model = world.to_gazebo().insert_model(modified_sdf_file,
                                                  initial_pose,
                                                  model_name)
        if not ok_model:
            raise RuntimeError('Failed to insert ' + model_name)

        """
        Get the model and Initialize the base class
        """
        model = world.get_model(model_name)
        model_wrapper.ModelWrapper.__init__(self, model=model)