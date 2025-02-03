"use client";

import { Button } from "@/components/ui/button";
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
import { Input } from "@/components/ui/input";
import { zodResolver } from "@hookform/resolvers/zod";
import axios from "axios";
import { Loader } from "lucide-react";
import { useRouter } from "next/navigation";
import { useState } from "react";
import { useForm } from "react-hook-form";
import toast from "react-hot-toast";
import { z } from "zod";

const LoginFormSchema = z.object({
  username: z
    .string()
    .email("Email không hợp lệ")
    .min(1, "Email không được để trống"),
  password: z.string().min(6, "Mật khẩu phải có ít nhất 6 ký tự"),
});

type LoginFormValues = z.infer<typeof LoginFormSchema>;

const LoginForm = () => {
  const [loading, setLoading] = useState(false);
  const router = useRouter();

  const form = useForm<LoginFormValues>({
    resolver: zodResolver(LoginFormSchema),
    mode: "onBlur",
  });

  async function onSubmit(data: LoginFormValues) {
    setLoading(true);
    try {
      const response = await axios.post(`/api/auth/login`, data);
      if (response?.data.token) {
        localStorage.setItem("token", response.data.token);
        toast.success("Đăng Nhập Thành Công");
        router.push("/select-node");
      }
    } catch (error) {
      console.error(error);
      toast.error("Tên đăng nhập hoặc mật khẩu không đúng");
    } finally {
      setLoading(false);
    }
  }

  return (
    <Card className="w-[24em] bg-white/30 backdrop-blur-lg rounded-xl shadow-xl">
      <CardHeader className="space-y-2 text-center">
        <CardTitle className="text-3xl font-bold text-gray-900">
          Đăng Nhập
        </CardTitle>
        <CardDescription className="text-gray-700">
          QuangHieu_IoT
        </CardDescription>
      </CardHeader>
      <CardContent>
        <Form {...form}>
          <form onSubmit={form.handleSubmit(onSubmit)}>
            <FormField
              control={form.control}
              name="username"
              render={({ field }) => (
                <FormItem className="mb-2">
                  <FormLabel>Email</FormLabel>
                  <FormControl>
                    <Input
                      type="email"
                      placeholder="example@mail.com"
                      {...field}
                    />
                  </FormControl>
                  <FormMessage />
                </FormItem>
              )}
            />
            <FormField
              control={form.control}
              name="password"
              render={({ field }) => (
                <FormItem>
                  <FormLabel>Mật Khẩu</FormLabel>
                  <FormControl>
                    <Input
                      type="password"
                      placeholder="Nhập mật khẩu"
                      {...field}
                    />
                  </FormControl>
                  <FormMessage />
                </FormItem>
              )}
            />
            <div className="mt-6">
              <Button
                type="submit"
                className="w-full bg-gray-900 text-white py-2 rounded-lg hover:bg-gray-800"
                disabled={loading}
              >
                {loading && <Loader className="mr-2 h-4 w-4 animate-spin" />}
                Đăng Nhập
              </Button>
            </div>
          </form>
        </Form>
      </CardContent>
    </Card>
  );
};

export default LoginForm;
