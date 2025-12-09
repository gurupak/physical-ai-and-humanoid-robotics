# API Contracts: Not Applicable

**Feature**: 006-digital-twin-chapter  
**Date**: 2025-12-07

## Why No Contracts?

This feature is **educational content** (book chapter), not a software system with APIs. The `/contracts/` directory typically contains:
- OpenAPI/Swagger specs for REST APIs
- GraphQL schemas
- Proto files for gRPC services
- Message queue contracts

None of these apply to static educational content.

---

## What This Feature Does Provide

Instead of API contracts, this feature defines:

### 1. Content Structure Contract

See [data-model.md](../data-model.md) for:
- ChapterMetadata interface (frontmatter schema)
- SubChapter interface (section structure)
- Diagram interface (Mermaid diagram definitions)
- CodeExample interface (code snippet metadata)
- Exercise interface (hands-on activity structure)

These "contracts" define how educational content is structured in MDX files.

### 2. Educational Interface

Readers interact with this chapter through:
- **Docusaurus UI**: Navigation, search, dark mode
- **MDX Content**: Text, diagrams, code examples, admonitions
- **External Tools**: Gazebo, Unity, ROS2 (not part of this project)

No API endpoints, no HTTP requests, no data exchange protocols.

---

## Future API Integration

If the book platform later adds features like:
- User progress tracking (read chapters, completed exercises)
- Personalized content recommendations
- Chatbot for Q&A

Those features would have API contracts, but they belong to **separate feature specs**:
- `specs/###-user-progress-api/contracts/`
- `specs/###-chatbot-api/contracts/`

This chapter (006-digital-twin-chapter) remains API-free: pure static educational content delivered via Docusaurus.

---

## Contract-Like Entities

While not "APIs", this chapter does define structured data:

### ROS2 Message Formats (Educational Reference)

These are **examples** readers will learn about, not API contracts:

**sensor_msgs/PointCloud2** (LiDAR):
```python
{
  "header": {"stamp": ..., "frame_id": "lidar_link"},
  "height": 1,
  "width": 65536,
  "fields": [{"name": "x", ...}, {"name": "y", ...}, ...],
  "data": bytes([...])
}
```

**sensor_msgs/Imu**:
```python
{
  "header": {...},
  "angular_velocity": {"x": ..., "y": ..., "z": ...},
  "linear_acceleration": {"x": ..., "y": ..., "z": ...}
}
```

These are **documented in data-model.md**, not here, because they're educational content, not API contracts for this project.

---

## Conclusion

The `/contracts/` directory exists for consistency with project structure template, but is intentionally empty for this feature. All structured definitions are in [data-model.md](../data-model.md).

**TL;DR**: Educational content = no APIs = no contracts. ✅
