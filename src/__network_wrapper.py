import os
import warnings
from dataclasses import dataclass, field
from input_data_class import NodeDetails

@dataclass
class NetworkDetails(NodeDetails):
    """Data class containing node details

    Args:
        is_path_hypergraph(optional): True if the defined graph is a path of hypergraph
            (a hypergraph with only one path like o-[o,o,o]-o). Defaults to False.
        he_dicts: dictionary of hyperedge (key: list of hyper edge's names, value : list of nodes in he.
            (hyperedge is a set of nodes which consists complete graph.)
        source_he: hyper edge's name which contains source_node
        destination_he: hyper edge's name which contains destination_node
        outbound_path_in_he : Sequence of nodes in he from source node to desitnation, in terms of node names.
            Only needed if the graph is a path hypergraph.
        inbound_path_in_he : Sequence of nodes in he from destination to source,in terms of node names. 
            If not specified, reverse of outboud is assumed.
        """
    is_path_hypergraph: bool = True
    he_dicts: dict = field(default_factory=dict) 
    outbound_path_in_he: dict = field(default_factory=dict) 
    inbound_path_in_he: dict = field(default_factory=dict) 
    source_node: str | None = None
    destination_node: str | None = None

    def __post_init__(self):
        self.is_path_graph = False

        if self.outbound_path:
            pass

        if self.inbound_path:
            pass
            
        super().__post_init__()
        
        else:
            warnings.warn(
                """The specified graph is not a path hypergraph.
                Some features may be limited, especially in the output file"""
            )
        return

        if self.is_path_graph:
            assert all(node in self.node_names for node in self.outbound_path), """
            One or more nodes in the specified outbound path
            (sequense of nodes from source to destination) cannot be found in
            the node name list. If the graph is not a path graph,
            set is_path_graph to False."""
            assert all(node in self.outbound_path for node in self.node_names), """
            Not all nodes appear in the specified outbound path
            (sequense of nodes from source to destination).
            If the graph is not a path graph, set is_path_graph to False."""
            for node in self.holdover_nodes:
                assert node in self.node_names, """
                Node {} in holdover nodes is not in the defined set of nodes.""".format(
                    node
                )
            if self.inbound_path:
                assert self.inbound_path == self.outbound_path[::-1]
            else:
                self.inbound_path = self.outbound_path[::-1]
            if not self.source_node:
                self.source_node = self.outbound_path[0]
            else:
                assert self.source_node in self.node_names, """
                The specified source node is not in the defined set of nodes."""
            if not self.destination_node:
                self.destination_node = self.outbound_path[-1]
            else:
                assert self.destination_node in self.node_names, """
                The specified destination node is not in the defined set of nodes."""


def main():
    net = NetworkDetails(['LEO','GTO','GEO'])
    print(net.is_path_graph)

if __name__ == "__main__":
    main()
