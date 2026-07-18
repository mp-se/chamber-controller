import { fileURLToPath, URL } from 'node:url'
import viteCompression from 'vite-plugin-compression'
import { defineConfig } from 'vite'
import vue from '@vitejs/plugin-vue'
import { readFileSync } from 'node:fs'

// Read package.json for version
const pkg = JSON.parse(readFileSync('./package.json', 'utf-8'))

// https://vitejs.dev/config/
export default defineConfig({
  test: {
    globals: true,
    environment: 'happy-dom',
    setupFiles: ['src/tests/setup.js'],
    deps: {
      inline: ['@mp-se/espframework-ui-components']
    },
    coverage: {
      provider: 'v8',
      reporter: ['text', 'json', 'html'],
      include: ['src/**/*.{js,vue}'],
      exclude: ['node_modules/', 'src/**/*.test.js', 'src/tests/**', 'src/main.js']
    }
  },
  plugins: [
    vue({
      template: {
        compilerOptions: {
          // Remove comments and whitespace in production
          hoistStatic: true,
          cacheHandlers: true
        }
      }
    }),
    viteCompression({
      algorithm: 'gzip',
      threshold: 1024,
      deleteOriginFile: false
    })
  ],
  resolve: {
    alias: {
      '@': fileURLToPath(new URL('./src', import.meta.url))
    }
  },
  define: {
    __VUE_OPTIONS_API__: false, // Disable Options API if not used
    __VUE_PROD_DEVTOOLS__: false,
    // Environment-specific settings
    __APP_VERSION__: JSON.stringify(pkg.version),
    __BUILD_TIME__: JSON.stringify(new Date().toISOString())
  },
  build: {
    minify: 'terser',
    cssCodeSplit: false,
    sourcemap: false,
    target: 'es2015',
    chunkSizeWarningLimit: 1000, // Disable chunk size warning (default is 500kB)
    terserOptions: {
      compress: {
        drop_console: false, // Keep console for debugging
        drop_debugger: true,
        passes: 2,
        unsafe: false, // Disable unsafe optimizations that might break code
        unsafe_comps: false,
        unsafe_Function: false,
        unsafe_math: false,
        unsafe_methods: false,
        unsafe_proto: false,
        unsafe_regexp: false,
        unsafe_undefined: false,
        side_effects: false
      },
      mangle: {
        properties: false // Disable property mangling to avoid breaking Vue
      },
      format: {
        comments: false
      }
    },
    rollupOptions: {
      treeshake: true, // Use default tree-shaking instead of aggressive preset
      external: [
        'json-parse-even-better-errors'
      ],
      onwarn(warning, warn) {
        // Suppress eval warnings for formula calculations
        if (warning.code === 'EVAL' && warning.id?.includes('formula.js')) {
          return
        }
        // Suppress warnings for externalized dependencies
        if (warning.code === 'UNRESOLVED_IMPORT') {
          return
        }
        warn(warning)
      },
      output: {
        inlineDynamicImports: true,
        entryFileNames: `assets/[name].js`,
        chunkFileNames: `assets/[name].js`,
        assetFileNames: `assets/[name].[ext]`,
        manualChunks: undefined
      }
    }
  },
  css: {
    preprocessorOptions: {
      scss: {
        charset: false
      }
    }
    // PostCSS config is now handled by postcss.config.js
  }
})
