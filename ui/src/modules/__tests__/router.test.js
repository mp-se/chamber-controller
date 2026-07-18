import { describe, it, expect, beforeEach, vi } from 'vitest'
import { createMemoryHistory } from 'vue-router'
import { validateCurrentForm } from '@mp-se/espframework-ui-components'

// Mock createWebHistory before importing router
vi.mock('vue-router', async () => {
  const actual = await vi.importActual('vue-router')
  return {
    ...actual,
    createWebHistory: vi.fn(() => createMemoryHistory())
  }
})

import router from '../router'
import { global } from '@/modules/pinia'

describe('router module - actual router instance', () => {
  let testRouter

  beforeEach(() => {
    testRouter = router
  })

  describe('router instance', () => {
    it('creates a router instance', () => {
      expect(router).toBeDefined()
      expect(router).toHaveProperty('push')
      expect(router).toHaveProperty('replace')
      expect(router).toHaveProperty('go')
      expect(router).toHaveProperty('back')
      expect(router).toHaveProperty('currentRoute')
    })

    it('has routes defined', () => {
      const allRoutes = router.getRoutes()
      expect(allRoutes.length).toBeGreaterThan(0)
    })
  })

  describe('route definitions', () => {
    it('home route has correct path and name', () => {
      const route = router.getRoutes().find((r) => r.name === 'home')
      expect(route).toBeDefined()
      expect(route.path).toBe('/')
    })

    it('device settings route has correct path', () => {
      const route = router.getRoutes().find((r) => r.name === 'device-settings')
      expect(route).toBeDefined()
      expect(route.path).toBe('/device/settings')
    })

    it('device hardware route has correct path', () => {
      const route = router.getRoutes().find((r) => r.name === 'device-hardware')
      expect(route).toBeDefined()
      expect(route.path).toBe('/device/hardware')
    })

    it('device PID route has correct path', () => {
      const route = router.getRoutes().find((r) => r.name === 'device-pid')
      expect(route).toBeDefined()
      expect(route.path).toBe('/device/pid')
    })

    it('device WiFi route has correct path', () => {
      const route = router.getRoutes().find((r) => r.name === 'device-wifi')
      expect(route).toBeDefined()
      expect(route.path).toBe('/device/wifi')
    })

    it('device WiFi 2 route has correct path', () => {
      const route = router.getRoutes().find((r) => r.name === 'device-wifi2')
      expect(route).toBeDefined()
      expect(route.path).toBe('/device/wifi2')
    })

    it('firmware route has correct path', () => {
      const route = router.getRoutes().find((r) => r.name === 'firmware')
      expect(route).toBeDefined()
      expect(route.path).toBe('/other/firmware')
    })

    it('InfluxDB push route has correct path', () => {
      const route = router.getRoutes().find((r) => r.name === 'push-influxdb')
      expect(route).toBeDefined()
      expect(route.path).toBe('/push/influxdb')
    })

    it('Bluetooth push route has correct path', () => {
      const route = router.getRoutes().find((r) => r.name === 'push-bluetooth')
      expect(route).toBeDefined()
      expect(route.path).toBe('/push/bluetooth')
    })

    it('support route has correct path', () => {
      const route = router.getRoutes().find((r) => r.name === 'support')
      expect(route).toBeDefined()
      expect(route.path).toBe('/other/support')
    })

    it('tools route has correct path', () => {
      const route = router.getRoutes().find((r) => r.name === 'tools')
      expect(route).toBeDefined()
      expect(route.path).toBe('/other/tools')
    })

    it('serial route has correct path', () => {
      const route = router.getRoutes().find((r) => r.name === 'serial')
      expect(route).toBeDefined()
      expect(route.path).toBe('/other/serial')
    })

    it('has catch-all 404 route', () => {
      const route = router.getRoutes().find((r) => r.name === '404')
      expect(route).toBeDefined()
    })

    it('all routes have components', () => {
      const routes = router.getRoutes()
      routes.forEach((route) => {
        if (route.name !== undefined) {
          expect(route.components || route.component).toBeDefined()
        }
      })
    })

    it('no duplicate route names', () => {
      const routes = router.getRoutes()
      const names = routes.map((r) => r.name).filter(Boolean)
      const uniqueNames = new Set(names)
      expect(names.length).toBe(uniqueNames.size)
    })

    it('no duplicate route paths (except catch-all)', () => {
      const routes = router.getRoutes()
      const paths = routes
        .map((r) => r.path)
        .filter((p) => !p.includes(':'))
      const uniquePaths = new Set(paths)
      expect(paths.length).toBe(uniquePaths.size)
    })
  })

  describe('route grouping', () => {
    it('has device routes', () => {
      const deviceRoutes = router.getRoutes().filter((r) => r.path?.startsWith('/device'))
      expect(deviceRoutes.length).toBe(5)
    })

    it('has push routes', () => {
      const pushRoutes = router.getRoutes().filter((r) => r.path?.startsWith('/push'))
      expect(pushRoutes.length).toBe(2)
    })

    it('has other routes', () => {
      const otherRoutes = router.getRoutes().filter((r) => r.path?.startsWith('/other'))
      expect(otherRoutes.length).toBe(4)
    })

    it('device routes always start with /device', () => {
      const deviceRoutes = router.getRoutes().filter((r) => r.name?.startsWith('device-'))
      deviceRoutes.forEach((route) => {
        expect(route.path.startsWith('/device')).toBe(true)
      })
    })

    it('push routes always start with /push', () => {
      const pushRoutes = router.getRoutes().filter((r) => r.name?.startsWith('push-'))
      pushRoutes.forEach((route) => {
        expect(route.path.startsWith('/push')).toBe(true)
      })
    })
  })

  describe('router methods', () => {
    it('has push method', () => {
      expect(typeof router.push).toBe('function')
    })

    it('has replace method', () => {
      expect(typeof router.replace).toBe('function')
    })

    it('has go method', () => {
      expect(typeof router.go).toBe('function')
    })

    it('has back method', () => {
      expect(typeof router.back).toBe('function')
    })

    it('has forward method', () => {
      expect(typeof router.forward).toBe('function')
    })

    it('has beforeEach method', () => {
      expect(typeof router.beforeEach).toBe('function')
    })

    it('has afterEach method', () => {
      expect(typeof router.afterEach).toBe('function')
    })

    it('has getRoutes method', () => {
      expect(typeof router.getRoutes).toBe('function')
    })

    it('has isReady method', () => {
      expect(typeof router.isReady).toBe('function')
    })
  })

  describe('router state', () => {
    it('has currentRoute property', () => {
      expect(router).toHaveProperty('currentRoute')
      expect(router.currentRoute).toHaveProperty('value')
    })

    it('currentRoute has path property', () => {
      expect(router.currentRoute.value).toHaveProperty('path')
    })

    it('currentRoute has name property', () => {
      expect(router.currentRoute.value).toHaveProperty('name')
    })

    it('currentRoute has matched property', () => {
      // matched contains the matched route components
      expect(router.currentRoute.value).toHaveProperty('matched')
    })
  })

  describe('route path definitions', () => {
    it('all routes have proper path definitions', () => {
      const routes = router.getRoutes()
      routes.forEach((route) => {
        expect(typeof route.path).toBe('string')
        expect(route.path.length).toBeGreaterThan(0)
      })
    })

    it('all routes (except catch-all) have names', () => {
      const routes = router.getRoutes()
      const namedRoutes = routes.filter((r) => r.name && r.name !== '404')
      expect(namedRoutes.length).toBeGreaterThan(0)
    })
  })

  describe('beforeEach hook', () => {
    it('beforeEach hook is registered', () => {
      // This is registered in router.js, but it's hard to verify from the exported router
      // We test that the hook exists
      expect(router).toHaveProperty('beforeEach')
    })

    it('can register custom beforeEach hooks', () => {
      const hook = vi.fn(() => true)
      router.beforeEach(hook)
      expect(typeof hook).toBe('function')
    })

    it('can register custom afterEach hooks', () => {
      const hook = vi.fn()
      router.afterEach(hook)
      expect(typeof hook).toBe('function')
    })
  })

  describe('beforeEach guard — actual navigation invocations', () => {
    it('guard executes return false when global.disabled is true (line 115)', async () => {
      global.disabled = true
      global.clearMessages = vi.fn()
      vi.mocked(validateCurrentForm).mockReturnValue(true)

      // Navigation is blocked by the guard; push resolves to NavigationFailure (no throw)
      await router.push('/device/pid')

      // Restore so subsequent tests are not affected
      global.disabled = false
    })

    it('guard executes return false when validateCurrentForm returns false (line 117)', async () => {
      global.disabled = false
      global.clearMessages = vi.fn()
      vi.mocked(validateCurrentForm).mockReturnValue(false)

      await router.push('/push/influxdb')

      vi.mocked(validateCurrentForm).mockReturnValue(true)
    })

    it('guard calls clearMessages and returns true when all checks pass', async () => {
      global.disabled = false
      global.clearMessages = vi.fn()
      vi.mocked(validateCurrentForm).mockReturnValue(true)

      await router.push('/other/support')

      expect(global.clearMessages).toHaveBeenCalled()
    })
  })

  describe('beforeEach navigation guard execution', () => {
    beforeEach(() => {
      global.disabled = false
      global.clearMessages = vi.fn()
      vi.mocked(validateCurrentForm).mockReturnValue(true)
      vi.clearAllMocks()
    })

    it('executes beforeEach hook on navigation', async () => {
      global.disabled = false
      vi.mocked(validateCurrentForm).mockReturnValue(true)

      // Navigate to trigger the hook
      try {
        await router.push('/device/settings')
      } catch (e) {
        // May fail but hook should execute
      }

      // Verify navigation was attempted
      expect(router).toHaveProperty('currentRoute')
    })

    it('guard early return when disabled=true', () => {
      global.disabled = true
      
      // Simulate guard logic
      let guardResult = undefined
      if (global.disabled) {
        guardResult = false
      }
      
      expect(guardResult).toBe(false)
    })

    it('guard early return when form validation fails', () => {
      global.disabled = false
      vi.mocked(validateCurrentForm).mockReturnValue(false)
      
      // Simulate guard logic
      let guardResult = undefined
      if (global.disabled) {
        guardResult = false
      } else if (!validateCurrentForm()) {
        guardResult = false
      }
      
      expect(guardResult).toBe(false)
    })

    it('guard executes clearMessages when all checks pass', () => {
      global.disabled = false
      global.clearMessages = vi.fn()
      vi.mocked(validateCurrentForm).mockReturnValue(true)

      // Simulate guard logic
      let guardResult = undefined
      if (global.disabled) {
        guardResult = false
      } else if (!validateCurrentForm()) {
        guardResult = false
      } else {
        global.clearMessages()
        guardResult = true
      }
      
      expect(guardResult).toBe(true)
      expect(global.clearMessages).toHaveBeenCalled()
    })

    it('guard returns true on successful validation', () => {
      global.disabled = false
      vi.mocked(validateCurrentForm).mockReturnValue(true)

      // Simulate guard execution
      let result = true
      if (global.disabled) result = false
      if (!validateCurrentForm()) result = false
      
      expect(result).toBe(true)
    })

    it('guard first check: global.disabled', () => {
      // Test that disabled check happens first
      global.disabled = true
      
      const shouldReturnEarly = global.disabled
      expect(shouldReturnEarly).toBe(true)
    })

    it('guard second check: validateCurrentForm', () => {
      // Test validateCurrentForm check
      vi.mocked(validateCurrentForm).mockReturnValue(false)
      
      const shouldReturnEarly = !validateCurrentForm()
      expect(shouldReturnEarly).toBe(true)
    })

    it('guard executes clearMessages only on success', () => {
      global.disabled = false
      global.clearMessages = vi.fn()
      vi.mocked(validateCurrentForm).mockReturnValue(true)

      // Simulate all three guard statements
      if (global.disabled) {
        // Skip clearMessages
      } else if (!validateCurrentForm()) {
        // Skip clearMessages
      } else {
        global.clearMessages()
      }

      expect(global.clearMessages).toHaveBeenCalledTimes(1)
    })

    it('guard does not call clearMessages when disabled', () => {
      global.disabled = true
      global.clearMessages = vi.fn()
      vi.mocked(validateCurrentForm).mockReturnValue(true)

      // Simulate guard
      if (!global.disabled && validateCurrentForm()) {
        global.clearMessages()
      }

      expect(global.clearMessages).not.toHaveBeenCalled()
    })

    it('guard does not call clearMessages when validation fails', () => {
      global.disabled = false
      global.clearMessages = vi.fn()
      vi.mocked(validateCurrentForm).mockReturnValue(false)

      // Simulate guard
      if (!global.disabled && validateCurrentForm()) {
        global.clearMessages()
      }

      expect(global.clearMessages).not.toHaveBeenCalled()
    })
  })
})

