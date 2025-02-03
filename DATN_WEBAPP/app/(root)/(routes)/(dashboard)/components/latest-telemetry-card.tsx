import { Card, CardContent, CardHeader, CardTitle } from "@/components/ui/card";
import { cn } from "@/lib/utils";
import { Loader } from "lucide-react";
import moment from "moment";

interface LatestTelemetryCardProps {
  loading: boolean;
  title: string;
  icon: any;
  data?: any;
  isBoolean?: boolean;
  booleanArr?: string[];
  isInteger?: boolean;
  unit?: string;
  children?: React.ReactNode;
  className?: any;
}

const LatestTelemetryCard: React.FC<LatestTelemetryCardProps> = ({
  loading,
  title,
  icon,
  data,
  unit,
  isBoolean = false,
  isInteger = false,
  children,
  className,
  booleanArr,
}) => {
  if (booleanArr?.length == 0) {
    booleanArr = ["Đang bật", "Đang tắt"];
  }
  return (
    <Card className={cn(`${className}`)}>
      <CardHeader className="flex flex-row items-center justify-between space-y-0">
        <CardTitle
          className="text-2xl font-sans font-bold text-transparent bg-clip-text bg-gradient-to-r"
          style={{
            backgroundImage:
              "linear-gradient(to right,rgb(62, 65, 89),rgb(91, 121, 184))",
          }}
        >
          {title}
        </CardTitle>
        {icon}
      </CardHeader>
      <CardContent>
        <div className="text-3xl font-bold flex gap-2" suppressHydrationWarning>
          {loading && <Loader className="mr-2 h-4 w-4 animate-spin" />}
          {!isBoolean && data && (
            <>
              {isInteger ? Math.round(data?.["value"]) : data?.["value"]}
              {unit}
            </>
          )}
          {isBoolean && data && (
            <p className={`text-xl font-bold ${data.color}`}>{data.text}</p>
          )}
        </div>
        <div
          className="text-sm from-muted-foreground text-gray-400"
          suppressHydrationWarning
        >
          Cập nhật {moment(data?.["ts"]).format("HH:mm:ss DD/MM/YYYY")}
        </div>
        {children}
      </CardContent>
    </Card>
  );
};

export default LatestTelemetryCard;
