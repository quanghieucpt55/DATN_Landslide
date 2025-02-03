import axios from "axios";
import { config } from "@/lib/config";

export const getTenantDeviceInfos = async (
  token: string,
  query: any
): Promise<any> => {
  const url = `https://${config.tbServer}/api/tenant/deviceInfos`;

  try {
    const response = await axios.get(url, {
      headers: {
        "X-Authorization": `Bearer ${token}`,
      },
      params: {
        pageSize: query.pageSize,
        page: query.page,
        deviceProfileId: query.deviceProfileId,
        sortProperty: query.sortProperty,
        sortOrder: query.sortOrder,
      },
    });
    return response.data;
  } catch (error) {
    console.error("Error fetching tenant device infos:", error);
    throw error;
  }
};
