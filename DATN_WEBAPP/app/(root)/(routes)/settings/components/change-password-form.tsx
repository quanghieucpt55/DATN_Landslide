"use client";

import { Button } from "@/components/ui/button";
import {
  Form,
  FormControl,
  FormField,
  FormItem,
  FormLabel,
  FormMessage,
} from "@/components/ui/form";
import { Input } from "@/components/ui/input";
import { thingsboard } from "@/lib/tbClient";
import { zodResolver } from "@hookform/resolvers/zod";
import { Loader } from "lucide-react";
import { redirect, useRouter } from "next/navigation";
import { useState } from "react";
import { useForm } from "react-hook-form";
import toast from "react-hot-toast";
import { z } from "zod";

const ChangePasswordFormSchema = z
  .object({
    currentPassword: z.string().min(6),
    newPassword: z.string().min(6),
    confirmNewPassword: z.string().min(6),
  })
  .refine((data) => data.newPassword === data.confirmNewPassword, {
    path: ["confirmNewPassword"],
    message: "Password do not match",
  });

type ChangePasswordFormValues = z.infer<typeof ChangePasswordFormSchema>;

const ChangePasswordForm = () => {
  const [loading, setLoading] = useState(false);

  const router = useRouter();

  const form = useForm<ChangePasswordFormValues>({
    resolver: zodResolver(ChangePasswordFormSchema),
    mode: "onSubmit",
  });

  async function onSubmit(data: ChangePasswordFormValues) {
    setLoading(true);
    const token = localStorage.getItem("token");
    if (!token) {
      redirect("/login");
    }
    const { confirmNewPassword, ...req } = data;
    await thingsboard
      .auth()
      .changePassword(token, req)
      .then(() => {
        toast.success("Đổi Mật Khẩu Thành Công");
        router.push("/");
      })
      .catch((error) => {
        toast.error(error?.response?.data.message);
      })
      .finally(() => {
        setLoading(false);
      });
  }

  return (
    <Form {...form}>
      <form
        onSubmit={form.handleSubmit(onSubmit)}
        className="space-y-8 max-w-md"
      >
        <div className="space-y-4">
          <FormField
            control={form.control}
            name="currentPassword"
            render={({ field }) => (
              <FormItem>
                <FormLabel>Mật khẩu hiện tại</FormLabel>
                <FormControl>
                  <Input
                    type="password"
                    placeholder="Nhập mật khẩu hiện tại"
                    {...field}
                  />
                </FormControl>
                <FormMessage />
              </FormItem>
            )}
          />
          <FormField
            control={form.control}
            name="newPassword"
            render={({ field }) => (
              <FormItem>
                <FormLabel>Mật khẩu mới</FormLabel>
                <FormControl>
                  <Input
                    type="password"
                    placeholder="Nhập mật khẩu mới"
                    {...field}
                  />
                </FormControl>
                <FormMessage />
              </FormItem>
            )}
          />
          <FormField
            control={form.control}
            name="confirmNewPassword"
            render={({ field }) => (
              <FormItem>
                <FormLabel>Xác nhận mật khẩu mới</FormLabel>
                <FormControl>
                  <Input
                    type="password"
                    placeholder="Nhập lại mật khẩu mới"
                    {...field}
                  />
                </FormControl>
                <FormMessage />
              </FormItem>
            )}
          />
        </div>
        <Button type="submit" className="w-full bg-gray-900" disabled={loading}>
          {loading && <Loader className="mr-2 h-4 w-4 animate-spin" />}
          Cập Nhật Mật Khẩu
        </Button>
      </form>
    </Form>
  );
};

export default ChangePasswordForm;
