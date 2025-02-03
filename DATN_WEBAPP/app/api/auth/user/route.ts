import { thingsboard } from "@/lib/tbClient";
import { NextResponse } from "next/server";

export async function POST(req: Request) {
  try {
    const data = await req.json();
    const { token } = data;
    const resp = await thingsboard.auth().getUser(token);
    return NextResponse.json(resp);
  } catch (err: any) {
    return NextResponse.json(err.response.data, {
      status: err.response.status,
    });
  }
}
