import { thingsboard } from "@/lib/tbClient";
import { NextResponse } from "next/server";

export async function POST(req: Request) {
  try {
    const data = await req.json();
    const { token, entityId, entityType, keys, startTs, endTs } = data;
    const resp = await thingsboard.telemetry().getTimeseries(
      token,
      {
        entityId,
        entityType,
      },
      {
        keys,
        startTs,
        endTs,
      }
    );
    return NextResponse.json(resp);
  } catch (err: any) {
    return NextResponse.json(err.response.data, {
      status: err.response.status,
    });
  }
}
