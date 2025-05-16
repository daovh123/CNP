####################################################
# DVrouter.py
# Name:
# HUID:
#####################################################

import json
from router import Router
from packet import Packet

INFINITY = 16  # Giá trị "vô cực" để xử lý count-to-infinity

class DVrouter(Router):
    """Distance vector routing protocol implementation."""

    def __init__(self, addr, heartbeat_time):
        Router.__init__(self, addr)  # Initialize base class - DO NOT REMOVE
        self.heartbeat_time = heartbeat_time
        self.last_time = 0

        self.ports = {}              # endpoint -> port
        self.neighbors = {}          # endpoint -> cost
        self.distance_vector = {}    # dest -> (cost, next_hop)
        self.routing_table = {}      # dest -> port
        self.neighbor_vectors = {}   # neighbor -> vector{dest: cost}

    def handle_packet(self, port, packet):
        if packet.is_traceroute:
            dst = packet.dst_addr
            if dst in self.routing_table:
                self.send(self.routing_table[dst], packet)
        else:
            # Routing packet
            try:
                received = json.loads(packet.content)
                sender = packet.src_addr
                self.neighbor_vectors[sender] = received

                updated = self._recalculate_routes()
                if updated:
                    self._broadcast_distance_vector()
            except Exception:
                return

    def handle_new_link(self, port, endpoint, cost):
        self.ports[endpoint] = port
        self.neighbors[endpoint] = cost

        # Khởi tạo distance vector của chính nó
        self.distance_vector[endpoint] = (cost, endpoint)
        self._recalculate_routes()
        self._broadcast_distance_vector()

    def handle_remove_link(self, port):
        neighbor = None
        for n, p in self.ports.items():
            if p == port:
                neighbor = n
                break

        if neighbor:
            del self.ports[neighbor]
            del self.neighbors[neighbor]
            self.neighbor_vectors.pop(neighbor, None)

            # Xoá các đường đi liên quan
            to_delete = []
            for dest, (cost, nhop) in self.distance_vector.items():
                if nhop == neighbor:
                    to_delete.append(dest)
            for d in to_delete:
                self.distance_vector[d] = (INFINITY, None)

            self._recalculate_routes()
            self._broadcast_distance_vector()

    def handle_time(self, time_ms):
        if time_ms - self.last_time >= self.heartbeat_time:
            self.last_time = time_ms
            self._broadcast_distance_vector()

    def _broadcast_distance_vector(self):
        vector_to_send = {}
        for dest, (cost, _) in self.distance_vector.items():
            vector_to_send[dest] = cost

        msg = json.dumps(vector_to_send)
        for neighbor, port in self.ports.items():
            pkt = Packet(Packet.ROUTING, self.addr, neighbor, content=msg)
            self.send(port, pkt)

    def _recalculate_routes(self):
        updated = False
        new_dv = {}
        new_rt = {}

        all_dests = set(self.distance_vector.keys())
        for vec in self.neighbor_vectors.values():
            all_dests.update(vec.keys())
        all_dests.update(self.neighbors.keys())

        for dest in all_dests:
            if dest == self.addr:
                continue
            min_cost = INFINITY
            next_hop = None
            for neighbor in self.neighbors:
                link_cost = self.neighbors[neighbor]
                neighbor_vec = self.neighbor_vectors.get(neighbor, {})
                cost_to_dest = neighbor_vec.get(dest, INFINITY)
                total_cost = link_cost + cost_to_dest
                if total_cost < min_cost:
                    min_cost = total_cost
                    next_hop = neighbor

            if dest in self.neighbors and self.neighbors[dest] < min_cost:
                min_cost = self.neighbors[dest]
                next_hop = dest

            new_dv[dest] = (min_cost, next_hop)
            if next_hop in self.ports:
                new_rt[dest] = self.ports[next_hop]

        if new_dv != self.distance_vector:
            self.distance_vector = new_dv
            self.routing_table = new_rt
            updated = True

        return updated

    def __repr__(self):
        return f"DVrouter(addr={self.addr}, dv={self.distance_vector})"
