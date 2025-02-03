import { thingsboard } from "@/lib/tbClient";
import { NextResponse } from "next/server";
import { TbEntity } from "thingsboard-api-client";

export async function POST(req: Request) {
  try {
    const data = await req.json();
    const { token, deviceID_GW, payload } = data;
    const resp = await thingsboard.telemetry().saveEntityAttributesV2(
      token,
      {
        entityId: deviceID_GW,
        entityType: TbEntity.DEVICE,
        scope: "SHARED_SCOPE",
      },
      payload
    );
    return NextResponse.json(resp);
  } catch (err: any) {
    return NextResponse.json(err.response.data, {
      status: err.response.status,
    });
  }
}
