"use client";

import axios from "axios";
import { useEffect, useState } from "react";
import { config } from "@/lib/config";
import { useRouter, redirect } from "next/navigation";
import {
  Card,
  CardContent,
  CardDescription,
  CardHeader,
  CardTitle,
} from "@/components/ui/card";
import {
  Form,
  FormControl,
  FormField,
  FormItem,
  FormLabel,
  FormMessage,
} from "@/components/ui/form";
import { Button } from "@/components/ui/button";

const deviceProfileId = config.deviceProfileId;
const SelectNodePage = () => {
  const [nodes, setNodes] = useState([]);
  const [error, setError] = useState("");
  const [loading, setLoading] = useState(true);
  const router = useRouter();

  useEffect(() => {
    const token = localStorage.getItem("token");
    if (!token) {
      redirect("/login");
    }
    const fetchNodes = async () => {
      setLoading(true);
      await axios
        .post(`/api/tenant/deviceInfos`, {
          token,
          deviceProfileId: deviceProfileId,
          pageSize: 100,
          page: 0,
          sortProperty: "name",
          sortOrder: "ASC",
        })
        .then((response) => {
          const devices = response.data.data;
          if (devices && devices.length > 0) {
            const nodes = devices
              .filter((device: any) => !device.additionalInfo?.gateway)
              .map((device: any) => ({
                name: device.name,
                id: device.id.id,
              }));

            if (nodes.length > 0) {
              setNodes(nodes);
            }
          } else {
            setError("Không tìm thấy thiết bị nào.");
          }
        })
        .catch((err) => {
          setError("Đã xảy ra lỗi khi tải danh sách thiết bị.");
        })
        .finally(() => {
          setLoading(false);
        });
    };

    fetchNodes();
  }, [router]);

  const handleNodeClick = (deviceId: string, nodeName: string) => {
    const nodeNameEncoded = encodeURIComponent(nodeName);
    router.push(`/${deviceId}?name=${nodeNameEncoded}`);
  };

  if (loading) {
    return (
      <div className="w-full h-screen flex items-center justify-center bg-transparent">
        <div className="flex flex-col items-center">
          <div className="loader border-t-4 border-whitr-500 rounded-full w-16 h-16 animate-spin"></div>
          <p className="mt-4 text-2xl font-extrabold text-white">Đang tải...</p>
        </div>
      </div>
    );
  }

  if (error) {
    return <div className="text-center text-red-500 text-lg mt-5">{error}</div>;
  }

  return (
    <CardHeader className="w-full h-screen flex flex-col items-center justify-center">
      <CardTitle className="text-4xl text-blue-100 font-bold mb-5 text-center">
        Chọn Node
      </CardTitle>
      <CardContent className="flex flex-col gap-4 w-1/2">
        {nodes.map((node: any) => (
          <Button
            key={node.id}
            className="bg-white/5 text-2xl font-extrabold text-gray-200 backdrop-blur-lg px-4 py-2 rounded-lg shadow-xl"
            onClick={() => handleNodeClick(node.id, node.name)}
          >
            {node.name}
          </Button>
        ))}
      </CardContent>
    </CardHeader>
  );
};

export default SelectNodePage;
