const express = require("express");
const cors = require("cors");
const { auth } = require("./auth");
const path = require("path");
require("dotenv").config({ path: path.join(__dirname, ".env") });

const app = express();
const PORT = process.env.PORT || 3008;

// CORS configuration
app.use(
  cors({
    origin: [
      "http://localhost:3000",
      "http://localhost:3002",
      "https://gurupak.github.io",
      "https://gurupak.github.io/physical-ai-and-humanoid-robotics/",
    ],
    credentials: true,
    methods: ["GET", "POST", "PUT", "DELETE", "OPTIONS"],
    allowedHeaders: ["Content-Type", "Authorization", "X-Requested-With"],
    exposedHeaders: ["Set-Cookie"],
    maxAge: 86400,
  }),
);

app.use(express.json());
app.use(express.urlencoded({ extended: true }));

// Health check endpoint
app.get("/api/health", (req, res) => {
  res.json({
    status: "ok",
    timestamp: new Date().toISOString(),
    message: "Better Auth server is running",
  });
});

// Session endpoint to get current user
app.get("/api/auth/session", async (req, res) => {
  try {
    const session = await auth.api.getSession({
      headers: req.headers,
    });
    res.json(session);
  } catch (error) {
    console.error("Session error:", error);
    res.status(401).json({ error: "Not authenticated" });
  }
});

// Protected route example
app.get("/api/user/progress", async (req, res) => {
  try {
    const session = await auth.api.getSession({ headers: req.headers });
    if (!session?.user) {
      return res.status(401).json({ error: "Not authenticated" });
    }

    // This would be your actual user progress data
    res.json({
      userId: session.user.id,
      progress: session.user.progress || {},
      completedModules: [],
      totalModules: 50,
      lastAccessed: session.session.createdAt,
    });
  } catch (error) {
    console.error("Progress error:", error);
    res.status(500).json({ error: "Failed to fetch progress" });
  }
});

// Update user progress
app.post("/api/user/progress", async (req, res) => {
  try {
    const session = await auth.api.getSession({ headers: req.headers });
    if (!session?.user) {
      return res.status(401).json({ error: "Not authenticated" });
    }

    const { module, progress } = req.body;

    // Here you would update the user's progress in the database
    // For now, we'll just return the updated progress
    res.json({
      success: true,
      message: `Progress updated for ${module}`,
      timestamp: new Date().toISOString(),
    });
  } catch (error) {
    console.error("Update progress error:", error);
    res.status(500).json({ error: "Failed to update progress" });
  }
});

// Update user profile
app.put("/api/user/profile", async (req, res) => {
  try {
    const session = await auth.api.getSession({ headers: req.headers });
    if (!session?.user) {
      return res.status(401).json({ error: "Not authenticated" });
    }

    const { firstName, lastName, email, phone, expertiseLevel } = req.body;
    console.log("Update profile request:", {
      firstName,
      lastName,
      email,
      phone,
      expertiseLevel,
    });

    // Validate required fields
    if (!firstName || !email) {
      return res
        .status(400)
        .json({ error: "First name and email are required" });
    }

    // Validate expertise level
    const validLevels = ["beginner", "intermediate", "advanced", "expert"];
    if (expertiseLevel && !validLevels.includes(expertiseLevel)) {
      return res.status(400).json({ error: "Invalid expertise level" });
    }

    // Update user in database
    const { Pool } = require("pg");
    const pool = new Pool({ connectionString: process.env.DATABASE_URL });

    const fullName = `${firstName}${lastName ? " " + lastName : ""}`;

    await pool.query(
      `UPDATE "user"
       SET name = $1,
           email = $2,
           phone = $3,
           expertise_level = $4,
           first_name = $5,
           last_name = $6,
           "updatedAt" = NOW()
       WHERE id = $7`,
      [
        fullName,
        email,
        phone,
        expertiseLevel,
        firstName,
        lastName,
        session.user.id,
      ],
    );

    // Get updated user data
    const result = await pool.query(
      'SELECT id, name, email, phone, expertise_level, first_name, last_name FROM "user" WHERE id = $1',
      [session.user.id],
    );

    await pool.end();

    console.log("Updated user data:", result.rows[0]);

    // Map snake_case to camelCase for frontend
    const userData = result.rows[0];
    const mappedUser = {
      id: userData.id,
      name: userData.name,
      email: userData.email,
      phone: userData.phone,
      expertiseLevel: userData.expertise_level,
      firstName: userData.first_name,
      lastName: userData.last_name,
    };

    res.json({
      success: true,
      user: mappedUser,
    });
  } catch (error) {
    console.error("Update profile error:", error);
    res.status(500).json({ error: "Failed to update profile" });
  }
});

// Mount Better Auth handlers at root since basePath is empty
app.all("*", async (req, res) => {
  try {
    // Convert Express request to Better Auth format
    const request = new Request(
      `${req.protocol}://${req.get("host")}${req.originalUrl}`,
      {
        method: req.method,
        headers: req.headers,
        body:
          req.method !== "GET" && req.method !== "HEAD"
            ? JSON.stringify(req.body)
            : undefined,
      },
    );

    const response = await auth.handler(request);

    res.status(response.status);
    response.headers.forEach((value, key) => {
      res.setHeader(key, value);
    });

    const body = await response.text();
    if (body) {
      res.send(body);
    } else {
      res.end();
    }
  } catch (error) {
    console.error("Auth handler error:", error);
    res.status(500).json({ error: "Internal server error" });
  }
});

// Error handling middleware
app.use((err, req, res, next) => {
  console.error(err.stack);
  res.status(500).json({
    error: "Something went wrong!",
    message:
      process.env.NODE_ENV === "development"
        ? err.message
        : "Internal server error",
  });
});

// 404 handler
app.use("*", (req, res) => {
  res.status(404).json({ error: "Route not found" });
});

app.listen(PORT, () => {
  console.log(`Better Auth server running on http://localhost:${PORT}`);
  console.log(`CORS enabled for: http://localhost:3000, http://localhost:3002`);
  console.log("Available endpoints:");
  console.log("  - POST /api/auth/sign-in-email");
  console.log("  - POST /api/auth/sign-up");
  console.log("  - POST /api/auth/sign-social/github");
  console.log("  - POST /api/auth/sign-social/google");
  console.log("  - GET  /api/auth/session");
  console.log("  - GET  /api/user/progress");
  console.log("  - POST /api/user/progress");
});
