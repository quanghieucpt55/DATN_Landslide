const SelecNodeLayout = ({ children }: { children: React.ReactNode }) => {
  return (
    <div
      style={{
        backgroundImage: "url('leaflet/images/background.jpg')",
        backgroundSize: "cover",
        backgroundPosition: "center",
        backgroundRepeat: "no-repeat",
      }}
    >
      {children}
    </div>
  );
};

export default SelecNodeLayout;
