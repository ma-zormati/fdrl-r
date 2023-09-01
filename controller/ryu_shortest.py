from ryu.base import app_manager
from ryu.controller import ofp_event
from ryu.controller.handler import MAIN_DISPATCHER, set_ev_cls
from ryu.ofproto import ofproto_v1_0
from ryu.lib.packet import packet, ethernet, ether_types


class ShortestPathController(app_manager.RyuApp):
    OFP_VERSIONS = [ofproto_v1_0.OFP_VERSION]

    def __init__(self, *args, **kwargs):
        super(ShortestPathController, self).__init__(*args, **kwargs)
        self.topology = {}  # Network topology represented as a dictionary

        # Define and populate the mac_to_dpid dictionary
        self.mac_to_dpid = {
            '00:00:00:00:00:01': 1,
            '00:00:00:00:00:02': 2,
            '00:00:00:00:00:03': 3
            # Add more MAC-DPID mappings as needed
        }

    @set_ev_cls(ofp_event.EventOFPPacketIn, MAIN_DISPATCHER)
    def _packet_in_handler(self, ev):
        msg = ev.msg
        datapath = msg.datapath
        ofproto = datapath.ofproto
        pkt = packet.Packet(msg.data)
        eth = pkt.get_protocol(ethernet.ethernet)

        if eth.ethertype == ether_types.ETH_TYPE_LLDP:
            return

        self.shortest_path_routing(datapath, eth, msg.in_port)

    def add_link(self, src_dpid, src_port, dst_dpid, dst_port):
        if src_dpid not in self.topology:
            self.topology[src_dpid] = {}
        self.topology[src_dpid][src_port] = (dst_dpid, dst_port)


    # Dijkstra's algorithm to calculate the shortest path
    def get_shortest_path(self, src_dpid, dst_dpid):
        visited = set()
        distance = {src_dpid: 0}
        path = {}

        while len(visited) < len(self.topology):
            min_dpid = None
            for dpid in self.topology:
                if dpid not in visited and (min_dpid is None or distance[dpid] < distance[min_dpid]):
                    min_dpid = dpid

            visited.add(min_dpid)
            for port, (dst_dpid, _) in self.topology[min_dpid].items():
                if dst_dpid not in visited:
                    new_distance = distance[min_dpid] + 1
                    if dst_dpid not in distance or new_distance < distance[dst_dpid]:
                        distance[dst_dpid] = new_distance
                        path[dst_dpid] = (min_dpid, port)

        path_to_dst = []
        while dst_dpid != src_dpid:
            prev_dpid, port = path[dst_dpid]
            path_to_dst.insert(0, (dst_dpid, port))
            dst_dpid = prev_dpid

        return path_to_dst


    def shortest_path_routing(self, datapath, eth, in_port):
        src_dpid = datapath.id
        src_mac = eth.src
        dst_mac = eth.dst

        if dst_mac in self.mac_to_dpid:
            dst_dpid = self.mac_to_dpid[dst_mac]
        else:
            return

        shortest_path = self.get_shortest_path(src_dpid, dst_dpid)
        if shortest_path:
            next_dpid, out_port = shortest_path[0]
            actions = [datapath.ofproto_parser.OFPActionOutput(out_port)]
            self.send_packet_out(datapath, actions, in_port)

    def send_packet_out(self, datapath, actions, in_port):
        ofproto = datapath.ofproto
        parser = datapath.ofproto_parser
        out = parser.OFPPacketOut(
            datapath=datapath, buffer_id=ofproto.OFP_NO_BUFFER,
            in_port=in_port, actions=actions)
        datapath.send_msg(out)


if __name__ == '__main__':
    app_manager.require_app('ryu.app.ofctl_rest')
    app_manager.run()
