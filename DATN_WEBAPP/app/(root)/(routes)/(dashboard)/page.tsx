"use client";

import { useRouter } from "next/navigation";
import { useEffect } from "react";

const SelectNodePage = () => {
  const router = useRouter();

  useEffect(() => {
    if (localStorage.getItem("token")) {
      router.push("/select-node");
    } else {
      router.push("/login");
    }
  }, [router]);

  return (
    <div className="flex justify-center items-center h-screen">
      <p>Đang chuyển hướng...</p>
    </div>
  );
};

export default SelectNodePage;
