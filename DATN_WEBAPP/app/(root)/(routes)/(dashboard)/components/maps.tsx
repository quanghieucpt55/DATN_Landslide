"use client";

import { MapContainer, Marker, Popup, TileLayer } from "react-leaflet";
import { icon } from "leaflet";

const currentIcon = icon({
  iconUrl: "/leaflet/images/marker-icon.png",
  shadowUrl: "/leaflet/images/marker-shadow.png",
  iconAnchor: [10, 42],
  popupAnchor: [0, -51],
});

const diffIcon = icon({
  iconUrl: "/leaflet/images/safe.png",
  shadowUrl: "/leaflet/images/marker-shadow.png",
  iconSize: [18, 28],
  iconAnchor: [6, 22],
  popupAnchor: [0, -51],
});

const nodes = [
  { name: "Node_1", latitude: 21.012619, longitude: 105.765991 },
  { name: "Node_2", latitude: 21.012684, longitude: 105.765765 },
  { name: "Node_3", latitude: 21.012793, longitude: 105.766265 },
];

const Maps: React.FC<{ selectedNodeName: string }> = ({ selectedNodeName }) => {
  const selectedNode = nodes.find((node) => node.name === selectedNodeName);
  const centerLatitude = selectedNode?.latitude || nodes[0].latitude;
  const centerLongitude = selectedNode?.longitude || nodes[0].longitude;

  return (
    <MapContainer
      center={[centerLatitude, centerLongitude]}
      zoom={100}
      style={{ height: "500px", width: "100%" }}
    >
      <TileLayer
        attribution='&copy; <a href="https://www.openstreetmap.org/copyright">OpenStreetMap</a> contributors'
        url="https://{s}.tile.openstreetmap.org/{z}/{x}/{y}.png"
      />
      {nodes.map((node) => (
        <Marker
          key={node.name}
          position={[node.latitude, node.longitude]}
          icon={node.name === selectedNodeName ? currentIcon : diffIcon}
        >
          <Popup>Vị trí của {node.name}</Popup>
        </Marker>
      ))}
    </MapContainer>
  );
};

export default Maps;
