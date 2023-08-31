// vite.config.ts
import { defineConfig } from "file:///workspaces/azure-search-openai/app/frontend/node_modules/vite/dist/node/index.js";
import react from "file:///workspaces/azure-search-openai/app/frontend/node_modules/@vitejs/plugin-react/dist/index.mjs";
var vite_config_default = defineConfig({
  plugins: [react()],
  build: {
    outDir: "../backend/static",
    emptyOutDir: true,
    sourcemap: true,
    rollupOptions: {
      output: {
        manualChunks: (id) => {
          if (id.includes("@fluentui/react-icons")) {
            return "fluentui-icons";
          } else if (id.includes("@fluentui/react")) {
            return "fluentui-react";
          } else if (id.includes("node_modules")) {
            return "vendor";
          }
        }
      }
    }
  },
  server: {
    proxy: {
      "/ask": "http://localhost:5000",
      "/chat": "http://localhost:5000"
    }
  }
});
export {
  vite_config_default as default
};
//# sourceMappingURL=data:application/json;base64,ewogICJ2ZXJzaW9uIjogMywKICAic291cmNlcyI6IFsidml0ZS5jb25maWcudHMiXSwKICAic291cmNlc0NvbnRlbnQiOiBbImNvbnN0IF9fdml0ZV9pbmplY3RlZF9vcmlnaW5hbF9kaXJuYW1lID0gXCIvd29ya3NwYWNlcy9henVyZS1zZWFyY2gtb3BlbmFpL2FwcC9mcm9udGVuZFwiO2NvbnN0IF9fdml0ZV9pbmplY3RlZF9vcmlnaW5hbF9maWxlbmFtZSA9IFwiL3dvcmtzcGFjZXMvYXp1cmUtc2VhcmNoLW9wZW5haS9hcHAvZnJvbnRlbmQvdml0ZS5jb25maWcudHNcIjtjb25zdCBfX3ZpdGVfaW5qZWN0ZWRfb3JpZ2luYWxfaW1wb3J0X21ldGFfdXJsID0gXCJmaWxlOi8vL3dvcmtzcGFjZXMvYXp1cmUtc2VhcmNoLW9wZW5haS9hcHAvZnJvbnRlbmQvdml0ZS5jb25maWcudHNcIjtpbXBvcnQgeyBkZWZpbmVDb25maWcgfSBmcm9tIFwidml0ZVwiO1xuaW1wb3J0IHJlYWN0IGZyb20gXCJAdml0ZWpzL3BsdWdpbi1yZWFjdFwiO1xuXG4vLyBodHRwczovL3ZpdGVqcy5kZXYvY29uZmlnL1xuZXhwb3J0IGRlZmF1bHQgZGVmaW5lQ29uZmlnKHtcbiAgICBwbHVnaW5zOiBbcmVhY3QoKV0sXG4gICAgYnVpbGQ6IHtcbiAgICAgICAgb3V0RGlyOiBcIi4uL2JhY2tlbmQvc3RhdGljXCIsXG4gICAgICAgIGVtcHR5T3V0RGlyOiB0cnVlLFxuICAgICAgICBzb3VyY2VtYXA6IHRydWUsXG4gICAgICAgIHJvbGx1cE9wdGlvbnM6IHtcbiAgICAgICAgICAgIG91dHB1dDoge1xuICAgICAgICAgICAgICAgIG1hbnVhbENodW5rczogaWQgPT4ge1xuICAgICAgICAgICAgICAgICAgICBpZiAoaWQuaW5jbHVkZXMoXCJAZmx1ZW50dWkvcmVhY3QtaWNvbnNcIikpIHtcbiAgICAgICAgICAgICAgICAgICAgICAgIHJldHVybiBcImZsdWVudHVpLWljb25zXCI7XG4gICAgICAgICAgICAgICAgICAgIH0gZWxzZSBpZiAoaWQuaW5jbHVkZXMoXCJAZmx1ZW50dWkvcmVhY3RcIikpIHtcbiAgICAgICAgICAgICAgICAgICAgICAgIHJldHVybiBcImZsdWVudHVpLXJlYWN0XCI7XG4gICAgICAgICAgICAgICAgICAgIH0gZWxzZSBpZiAoaWQuaW5jbHVkZXMoXCJub2RlX21vZHVsZXNcIikpIHtcbiAgICAgICAgICAgICAgICAgICAgICAgIHJldHVybiBcInZlbmRvclwiO1xuICAgICAgICAgICAgICAgICAgICB9XG4gICAgICAgICAgICAgICAgfVxuICAgICAgICAgICAgfVxuICAgICAgICB9XG4gICAgfSxcbiAgICBzZXJ2ZXI6IHtcbiAgICAgICAgcHJveHk6IHtcbiAgICAgICAgICAgIFwiL2Fza1wiOiBcImh0dHA6Ly9sb2NhbGhvc3Q6NTAwMFwiLFxuICAgICAgICAgICAgXCIvY2hhdFwiOiBcImh0dHA6Ly9sb2NhbGhvc3Q6NTAwMFwiXG4gICAgICAgIH1cbiAgICB9XG59KTtcbiJdLAogICJtYXBwaW5ncyI6ICI7QUFBc1QsU0FBUyxvQkFBb0I7QUFDblYsT0FBTyxXQUFXO0FBR2xCLElBQU8sc0JBQVEsYUFBYTtBQUFBLEVBQ3hCLFNBQVMsQ0FBQyxNQUFNLENBQUM7QUFBQSxFQUNqQixPQUFPO0FBQUEsSUFDSCxRQUFRO0FBQUEsSUFDUixhQUFhO0FBQUEsSUFDYixXQUFXO0FBQUEsSUFDWCxlQUFlO0FBQUEsTUFDWCxRQUFRO0FBQUEsUUFDSixjQUFjLFFBQU07QUFDaEIsY0FBSSxHQUFHLFNBQVMsdUJBQXVCLEdBQUc7QUFDdEMsbUJBQU87QUFBQSxVQUNYLFdBQVcsR0FBRyxTQUFTLGlCQUFpQixHQUFHO0FBQ3ZDLG1CQUFPO0FBQUEsVUFDWCxXQUFXLEdBQUcsU0FBUyxjQUFjLEdBQUc7QUFDcEMsbUJBQU87QUFBQSxVQUNYO0FBQUEsUUFDSjtBQUFBLE1BQ0o7QUFBQSxJQUNKO0FBQUEsRUFDSjtBQUFBLEVBQ0EsUUFBUTtBQUFBLElBQ0osT0FBTztBQUFBLE1BQ0gsUUFBUTtBQUFBLE1BQ1IsU0FBUztBQUFBLElBQ2I7QUFBQSxFQUNKO0FBQ0osQ0FBQzsiLAogICJuYW1lcyI6IFtdCn0K
