import { NextResponse } from "next/server";
import { getTenantDeviceInfos } from "../../../utils/thingsboardApi";

export async function POST(req: Request) {
  try {
    const { token, deviceProfileId, pageSize, page, sortProperty, sortOrder } =
      await req.json();

    if (!token || !deviceProfileId) {
      return NextResponse.json(
        { error: "Token hoặc deviceProfileId không được cung cấp." },
        { status: 400 }
      );
    }

    const query = {
      pageSize,
      page,
      deviceProfileId,
      sortProperty,
      sortOrder,
    };

    const resp = await getTenantDeviceInfos(token, query);

    return NextResponse.json(resp);
  } catch (err: any) {
    return NextResponse.json(
      {
        error: "Đã xảy ra lỗi khi lấy danh sách thiết bị.",
        details: err.message,
      },
      { status: err.response?.status || 500 }
    );
  }
}
