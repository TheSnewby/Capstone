import type { Metadata } from "next";
import { Geist, Geist_Mono } from "next/font/google";
import "./globals.css";

const geistSans = Geist({
  variable: "--font-geist-sans",
  subsets: ["latin"],
});

const geistMono = Geist_Mono({
  variable: "--font-geist-mono",
  subsets: ["latin"],
});

export const metadata: Metadata = {
  title: "SkyWeave",
  description: "Control autonomous UAV swarms with real-time telemetry and commands.",
  icons: {
    icon: "/favicon.ico",
  },
  openGraph: {
    title: "SkyWeave",
    description: "Control autonomous UAV swarms with real-time telemetry and commands.",
    type: "website",
    siteName: "SkyWeave",
  },
  twitter: {
    card: "summary_large_image",
    title: "SkyWeave",
    description: "Control autonomous UAV swarms with real-time telemetry and commands.",
  },
};

export const viewport = {
  themeColor: "#0a0d12",
};

export default function RootLayout({
  children,
}: Readonly<{
  children: React.ReactNode;
}>) {
  return (
    <html lang="en" className="bg-black text-white">
      <body className={`${geistSans.variable} ${geistMono.variable} antialiased`}>
        <div id="skyweave-root" className="min-h-screen w-full">
          {children}
        </div>
      </body>
    </html>
  );
}
