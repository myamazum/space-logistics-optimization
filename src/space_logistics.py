from input_data_class import InputData, NodeDetails
from initializer import InitMixin
from component_designer.component_designer_class import ComponentDesigner
from network_builder.network_builder_class import NetworkBuilder
from opt_model_builder.opt_model_builder_class import OptModelBuilder
from optimizer.optimizer_class import Optimizer

from input_data_class_wrapper import NodeDetailsV2

class SpaceLogistics(InitMixin):
    def __init__(self, input_data: InputData) -> None:
        """Class to connect all classes via composition.

        Arguments can be a class itself or an instance of a class,
        depending on the class being initialized.
        A class itself is passed when initialization of the class needs to be
        delayed and done by the class to which it is passed as an argument.

        Args:
            input_data: InputData dataclass containing data input from user
        """

        node_cfg = input_data.node
        if isinstance(node_cfg, NodeDetails) and not isinstance(node_cfg, NodeDetailsV2):
            node_cfg = NodeDetailsV2.from_parent(node_cfg)
        self.network_def = NetworkBuilder(..., node_details=node_cfg)

        self.initialize_attributes(input_data)
        self.comp_design = ComponentDesigner(input_data)
        #self.network_def = NetworkBuilder(input_data)
        #self.model_builder = OptModelBuilder(input_data, self.comp_design)
        self.optimizer = Optimizer(input_data, self.comp_design)




