import Navbar from "@/components/navbar";

const SetupLayout = ({ children }: { children: React.ReactNode }) => {
  return (
    <>
      <Navbar />
      <div className="container mx-auto px-8 my-6 w-full">{children}</div>
    </>
  );
};

export default SetupLayout;
