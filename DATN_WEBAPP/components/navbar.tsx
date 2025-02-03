"use client";

import { Button } from "@/components/ui/button";
import { thingsboard } from "@/lib/tbClient";
import Link from "next/link";
import { redirect, useRouter } from "next/navigation";
import { useEffect, useState } from "react";
import MainNav from "./main-nav";
import axios from "axios";

const Navbar = () => {
  const router = useRouter();
  const [profileInfo, setProfileInfo] = useState({}) as any;

  useEffect(() => {
    const token = localStorage.getItem("token");
    if (!token) {
      redirect("/login");
    }
    const getData = async () => {
      await axios
        .post(`/api/auth/user`, { token })
        .then((resp) => {
          setProfileInfo(resp.data);
        })
        .catch((error) => {
          localStorage.removeItem("token");
          router.push("/login");
        });
    };
    getData();
  }, [router]);

  return (
    <div className="border-b">
      <div className="flex h-16 items-center px-6">
        <MainNav />
        <div className="ml-auto flex items-center space-x-4 gap-2">
          <div className="flex flex-col text-right">
            <span>{profileInfo ? `Chào, ${profileInfo?.lastName}` : ``}</span>
          </div>
          <Link href={"/logout"}>
            <Button>Đăng Xuất</Button>
          </Link>
        </div>
      </div>
    </div>
  );
};

export default Navbar;
